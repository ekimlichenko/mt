#!/usr/bin/env python3
"""Report figures for docs/RESULTS.md -> docs/plots/*.png (reproducible, offline).

The eval bags (``eval_ok`` in .work/bag_index.json, 60 unique runs with >= 60 s of
RTK reference) are replayed in parallel through the same ``Pipeline`` as the ROS
node (``tram_backup_odometry.replay.run_bag``, default ``Params``, GNSS fed only
for the first ``--gnss-limit`` s) and scored with ``tools/evaluate.py`` against
the master-antenna RTK reference.  Aggregate tables come from
``results/<tag>/{per_bag,summary}.json`` (``tools/run_cv.py``) and the real-time
measurements from ``results/rt/<run>/`` (``scripts/measure.sh``).

Figures (``--only`` takes a comma list of these names):

  speed_example  reference vs estimated speed on a stop-to-stop segment of a
                 representative T2S run (+ error, driver notch) and a zoom on
                 braking to a stop with the raw bogie speeds;
  speed_error    CDF of |speed error| pooled over the eval bags per direction and
                 per-regime RMSE / bias (per_bag.json of --tag);
  along_track    along-track error vs route arc length (on the pathgraph), every
                 eval bag + median and p10-p90 band, T2S and S2T;
  trajectory     both routes, reference vs estimate for one T2S and one S2T run,
                 zooms on the start (G1 bridge) and the end (tail) of each run;
  per_bag        al_rmse, e3d_mean, drift_pct per eval bag (per_bag.json);
  ablations      medians of the ablation / GNSS-window variants (summary.json);
  realtime       latency CDF / timeline, CPU and RSS of the node in Docker;
  covariance     along/cross errors of one run with the reported +-2 sigma and
                 the per-bag share of errors inside the 95 % ellipse.

Representative runs are chosen automatically (full-length run of the direction
whose metrics are closest to the direction medians of --tag); override with
--t2s-bag / --s2t-bag / --speed-bag / --speed-t0 / --cov-bag.

    python tools/report_plots.py                       # all figures, docs/plots/
    python tools/report_plots.py --only realtime,ablations --jobs 4
    python tools/report_plots.py --cache .work/scratch_deliv/series   # reuse replays
"""
import argparse
import csv
import json
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402,F401
from matplotlib.patches import Patch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import evaluate as ev  # noqa: E402

sys.path.insert(0, ev.PKG_DIR)
ROOT = ev.ROOT
RESULTS = os.path.join(ROOT, 'results')
INDEX = os.path.join(ROOT, '.work', 'bag_index.json')
TOPIC_CMD = '/vehicle/driver_position_cmd'

FIGS = ('speed_example', 'speed_error', 'along_track', 'trajectory', 'per_bag', 'ablations', 'realtime',
        'covariance')
NEED_ALL = {'speed_error', 'along_track'}                 # need the replay of every eval bag
NEED_DETAIL = {'speed_example', 'trajectory', 'covariance'}  # need full outputs of a few bags

# ---------------------------------------------------------------- style
# categorical slots in fixed order (validated default palette): blue, orange, aqua
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'
C_DIR = {'T2S': C1, 'S2T': C2}
C_REF = '#9d9c96'        # reference (thick, neutral, under the estimate)
C_INK = '#1f1f1d'
C_MUTED = '#6b6a65'
C_MAP = '#dcdad3'
C_BAND = '#b9b8b1'
DPI = 110

plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.labelsize': 9, 'legend.fontsize': 8,
    'xtick.labelsize': 8, 'ytick.labelsize': 8, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': '#e4e3de', 'grid.linewidth': 0.7, 'axes.edgecolor': '#8a8984',
    'axes.titleweight': 'bold', 'figure.titleweight': 'bold', 'figure.titlesize': 12,
    'legend.frameon': True, 'legend.framealpha': 0.9, 'legend.edgecolor': '#d0cfc9',
    'savefig.facecolor': 'white', 'figure.facecolor': 'white',
})

AF_LABELS = {
    'B1_naive': 'B1: наивная (только колёса)',
    'F4_final': 'итоговая версия',
    'AF_trn_off': 'без TRN (привязка по уклонам)',
    'AF_tail_straight': 'прямой хвост после карты',
    'AF_nobias': 'без состояния смещения модели',
    'AF_nolook': 'без упреждения точки уклона',
    'AF_nosc': 'без коррекции s через P(v,s)',
    'AF_nopairs': 'без проверки выровненной пары',
    'AF_noinflate': 'без σ·√2 при двух тележках',
    'AF_scalefb': '+ масштаб TRN в скорость',
    'AF_noloops': 'без геометрии петель (loops.json)',
    'AF_noaiding': 'без коррекции по GNSS в пути',
    'G_limit0': 'GNSS: только первый фикс',
    'G_limit05': 'GNSS: окно 0.5 с',
    'G_limit1': 'GNSS: окно 1 с',
    'G_limit3': 'GNSS: окно 3 с',
    'G_limit5': 'GNSS: окно 5 с',
    'G_limit30': 'GNSS: окно 30 с',
}
AF_ORDER = ('B1_naive', 'F4_final', 'AF_trn_off', 'AF_tail_straight', 'AF_nobias', 'AF_nolook', 'AF_nosc',
            'AF_nopairs', 'AF_noinflate', 'AF_scalefb', 'AF_noloops', 'AF_noaiding', 'G_limit0', 'G_limit05',
            'G_limit1', 'G_limit3', 'G_limit5', 'G_limit30')
REGIME_RU = {'accel': 'разгон', 'brake': 'торможение', 'stopped': 'стоянка', 'cruise': 'движение\nбез разгона'}


def _save(fig, out_dir, name):
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=DPI, bbox_inches='tight')
    plt.close(fig)
    print('  wrote', os.path.relpath(path, ROOT))
    return path


def _note(ax, text, loc='upper left', fs=8):
    xy = {'upper left': (0.01, 0.97, 'left', 'top'), 'upper right': (0.99, 0.97, 'right', 'top'),
          'lower left': (0.01, 0.03, 'left', 'bottom'), 'lower right': (0.99, 0.03, 'right', 'bottom')}[loc]
    ax.text(xy[0], xy[1], text, transform=ax.transAxes, ha=xy[2], va=xy[3], fontsize=fs, color=C_INK,
            bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#d0cfc9', alpha=0.9), zorder=10)


# ---------------------------------------------------------------- replay (worker)
def replay_one(job):
    """Replay + evaluate one bag -> compact arrays (and full outputs when detail)."""
    bag, detail, gnss_limit = job
    from tram_backup_odometry.replay import run_bag, load_data
    from tram_backup_odometry.core.config import Params
    D = load_data(ev.cache_path(bag))
    # scored against the GNSS master antenna: publish that point, as tools/run_cv.build_params does
    # (the package default is base_link + 0.09 s velocity delay for the organisers' checker)
    p = Params(output_along_offset_m=0.0, output_z_offset_m=Params().z_offset_m, output_velocity_delay_s=0.0)
    est = run_bag(bag, params=p, data=D, gnss_limit_s=gnss_limit)
    R = ev.evaluate(bag, est, series=True, D=D)
    S = R.pop('series', {}) or {}
    f32 = lambda a: np.asarray(a, np.float32)  # noqa: E731
    out = dict(bag=bag, dir=R.get('dir'), detail=detail,
               metrics={k: v for k, v in R.items() if isinstance(v, (int, float, str, type(None)))})
    if len(S.get('tv', [])):
        out.update(tv=np.asarray(S['tv'], float), v_ref=f32(S['v_ref']), v_err=f32(S['v_err']))
    if R.get('dir') and len(S.get('tp', [])):
        rt = ev.routes()[R['dir']]
        on = np.asarray(S['on'], bool)
        sr = rt.project(S['xyz_ref'][on, :2])[0]
        out.update(s_ref=f32(sr), al_on=f32(S['al'][on]), L=float(rt.L))
    if detail:
        keep = ('t', 'v', 'x', 'y', 'z', 'yaw', 'has_pose', 'var_along', 'var_cross', 'diag_v_front', 'diag_v_rear',
                'diag_s_route', 'diag_off_map_m')
        out['est'] = {k: np.asarray(est[k]) for k in keep if k in est}
        out['series'] = {k: np.asarray(S[k]) for k in ('tp', 'xyz_ref', 'xyz_est', 'e2d', 'al', 'ct', 'on',
                                                        'ref_tp_all', 'ref_xyz_all') if k in S}
        c = D.get(TOPIC_CMD) or {}
        if len(c.get('t_hdr', [])):
            o = np.argsort(c['t_hdr'], kind='stable')
            out['cmd'] = (np.asarray(c['t_hdr'], float)[o], np.asarray(c['pos'], float)[o])
        # errors in the reported (yaw) frame, as evaluate's p_in95, with the reported variances
        if len(S.get('tp', [])):
            E = ev.as_estimate(est)
            pm = E['has_pose'] & np.isfinite(E['xyz']).all(axis=1)
            j, _ = ev.nearest(S['tp'], E['t'][pm])
            C = E['cov'][pm][j]
            dxy = S['xyz_est'][:, :2] - S['xyz_ref'][:, :2]
            cy, sy = np.cos(C[:, 0]), np.sin(C[:, 0])
            vc = np.where(np.isfinite(C[:, 2]), C[:, 2], 0.09)
            off = np.asarray(est.get('diag_off_map_m', np.zeros(len(est['t']))), float)
            ti, _ = ev.nearest(S['tp'], np.asarray(est['t'], float)[np.argsort(est['t'], kind='stable')])
            off_sorted = off[np.argsort(est['t'], kind='stable')]
            out['cov'] = dict(tp=S['tp'], ea=dxy[:, 0] * cy + dxy[:, 1] * sy, ec=-dxy[:, 0] * sy + dxy[:, 1] * cy,
                              va=C[:, 1], vc=vc, off=off_sorted[ti])
    return out


def run_replays(bags, detail_bags, jobs, gnss_limit, cache_dir=None):
    res, todo = {}, []
    for b in bags:
        det = b in detail_bags
        cp = os.path.join(cache_dir, '%s%s.pkl' % (b, '_d' if det else '')) if cache_dir else None
        if cp and os.path.exists(cp):
            with open(cp, 'rb') as f:
                res[b] = pickle.load(f)
            continue
        todo.append((b, det, gnss_limit))
    if todo:
        t0 = time.time()
        order = sorted(todo, key=lambda j: -_duration(j[0]))
        with ProcessPoolExecutor(max(1, jobs)) as ex:
            for r in ex.map(replay_one, order):
                res[r['bag']] = r
                if cache_dir:
                    os.makedirs(cache_dir, exist_ok=True)
                    with open(os.path.join(cache_dir, '%s%s.pkl' % (r['bag'], '_d' if r['detail'] else '')), 'wb') as f:
                        pickle.dump(r, f, protocol=pickle.HIGHEST_PROTOCOL)
        print('  replayed %d bags in %.0f s (%d jobs)' % (len(todo), time.time() - t0, jobs))
    return res


_INDEX = None


def _index():
    global _INDEX
    if _INDEX is None:
        with open(INDEX) as f:
            _INDEX = json.load(f)['bags']
    return _INDEX


def _duration(bag):
    return float(_index().get(bag, {}).get('duration_s', 0.0) or 0.0)


# ---------------------------------------------------------------- tag data
def load_per_bag(tag, antenna='master'):
    with open(os.path.join(RESULTS, tag, 'per_bag.json')) as f:
        rows = json.load(f)
    return [r for r in rows if r.get('eval_ok') and antenna in (r.get('metrics') or {})
            and r['metrics'][antenna].get('dir') in ('T2S', 'S2T')]


def _m(r, k, antenna='master'):
    v = r['metrics'][antenna].get(k)
    return np.nan if v is None else float(v)


def pick_representative(rows, direction):
    """Full-length run of the direction whose metrics are closest (log scale) to the direction medians."""
    R = [r for r in rows if r['metrics']['master']['dir'] == direction]
    keys = ('v_rmse', 'al_rmse', 'e3d_mean', 'drift_pct', 'ct_rmse', 'p_in95')
    med = {k: np.nanmedian([_m(r, k) for r in R]) for k in keys}
    dist_med = np.nanmedian([_m(r, 'dist') for r in R])
    best, best_sc = None, np.inf
    for r in R:
        if not _m(r, 'dist') >= 0.97 * dist_med or not _m(r, 'onmap_frac') >= 0.6:
            continue
        sc = sum(abs(np.log(max(_m(r, k), 1e-6) / med[k])) for k in keys)
        if sc < best_sc:
            best, best_sc = r['bag'], sc
    return best


# ---------------------------------------------------------------- 1. speed example
def _smooth(v, n=11):
    h = n // 2
    return np.convolve(np.r_[np.full(h, v[0]), v, np.full(h, v[-1])], np.ones(n) / n, mode='valid')


def _stop_segments(tv, vs, a):
    """Stop-to-stop segments: dicts with dep/arr stamps, vmax, braking start, cruise time, reference gap."""
    stopped = vs < 0.1
    runs, i, n = [], 0, len(vs)
    while i < n:                                 # stop runs of >= 3 s (reference gaps > 1.5 s split runs)
        if stopped[i]:
            j = i
            while j + 1 < n and stopped[j + 1] and tv[j + 1] - tv[j] < 1.5:
                j += 1
            if tv[j] - tv[i] >= 3.0:
                runs.append((i, j))
            i = j + 1
        else:
            i += 1
    dt_med = float(np.median(np.diff(tv)))
    segs = []
    for (i0, i1), (j0, j1) in zip(runs[:-1], runs[1:]):
        k = np.arange(i1, j0 + 1)
        vmax = float(vs[k].max())
        mov = k[vs[k] > 3.0]
        acc_end = tv[mov[np.argmax(a[mov] < 0.05)]] if len(mov) else tv[i1]
        nb = k[(a[k] > -0.05) & (tv[k] < tv[j0] - 2.0)]
        brake = tv[nb[-1]] if len(nb) else tv[j0] - 15.0
        mid = k[(tv[k] >= acc_end) & (tv[k] <= brake)]
        dip = float(vs[mid].min()) / vmax if len(mid) else 0.0
        cruise = float(np.sum((np.abs(a[mid]) < 0.12) & (vs[mid] > 0.6 * vmax)) * dt_med) if len(mid) else 0.0
        segs.append(dict(dep=tv[i1], arr=tv[j0], vmax=vmax, brake=brake, dip=dip, cruise=cruise,
                         gap=float(np.max(np.diff(tv[i0:j1 + 1])))))
    return segs


def find_speed_window(tv, v, max_len=160.0, min_len=90.0, pad=8.0):
    """Window of 1-3 consecutive stop-to-stop segments (starts and ends at a stop), <= max_len s,
    with the most balanced time in the four regimes.

    Returns (t_start, t_end, zoom_start, zoom_end); the zoom covers the braking of the fastest segment."""
    vs = _smooth(v)
    tt = tv - tv[0]
    a = np.gradient(_smooth(vs, 21), tt + np.arange(len(tt)) * 1e-9)
    segs = _stop_segments(tv, vs, a)
    best, best_sc = None, -np.inf
    for p in range(len(segs)):
        for c in (1, 2, 3):
            G = segs[p:p + c]
            if len(G) < c:
                break
            span = G[-1]['arr'] - G[0]['dep'] + 2 * pad
            if span > max_len:
                break
            if span < min_len or any(g['gap'] > 1.0 for g in G) or max(g['vmax'] for g in G) < 8.0:
                continue
            # balanced window: time in the least represented regime (accel / cruise / brake / stopped,
            # the regimes of the per-regime metrics), ties broken by length and top speed
            m = (tv >= G[0]['dep'] - pad) & (tv <= G[-1]['arr'] + pad)
            comp = dict.fromkeys(ev.REGIMES, 0.0)
            for a_, b_, g in _regime_runs(tv[m], v[m]):
                comp[g] += b_ - a_
            sc = min(comp.values()) + 0.05 * span + 0.2 * max(g['vmax'] for g in G)
            if sc > best_sc:
                best_sc = sc
                f = max(G, key=lambda g: g['vmax'])
                best = (G[0]['dep'] - pad, G[-1]['arr'] + pad, f['brake'] - 4.0, f['arr'] + 6.0)
    if best is None:                             # fallback: around the fastest sample
        k = int(np.argmax(vs))
        return tv[k] - max_len / 2, tv[k] + max_len / 2, tv[k], tv[k] + 40.0
    return best


REGIME_FILL = {'accel': '#d3efe4', 'brake': '#fbdccd', 'stopped': '#e6e5e0', 'cruise': '#dce8f8'}
REGIME_SHORT = {'accel': ('разгон', 'разг.'), 'brake': ('торможение', 'торм.'), 'stopped': ('стоянка', 'стоп'),
                'cruise': ('равномерно', 'равн.')}


def _regime_runs(tv, v, min_s=2.0):
    """Regimes of evaluate.regimes (the per-regime metrics) as runs; runs < min_s s merged into the previous."""
    r = ev.regimes(tv, v)
    runs = []
    for i in range(len(r)):
        if runs and runs[-1][2] == r[i]:
            runs[-1][1] = tv[i]
        else:
            runs.append([tv[i], tv[i], r[i]])
    out = []
    for a, b, g in runs:
        if out and (b - a < min_s or out[-1][2] == g):
            out[-1][1] = b
        else:
            out.append([a, b, g])
    return out


def fig_speed_example(R, out_dir, t0_override=None):
    tv, vr, ve = R['tv'], R['v_ref'], R['v_err']
    E = R['est']
    t_est = np.asarray(E['t'], float)
    o = np.argsort(t_est, kind='stable')
    t_est = t_est[o]
    if t0_override is not None:
        t_a = tv[0] + t0_override
        t_b = t_a + 150.0
        za, zb = t_b - 40.0, t_b
    else:
        t_a, t_b, za, zb = find_speed_window(tv, vr.astype(float))
    T0 = t_a

    def sel(t, a, b):
        return (t >= a) & (t <= b)

    fig = plt.figure(figsize=(15.5, 8.8))
    gs = fig.add_gridspec(4, 2, width_ratios=(2.1, 1.0), height_ratios=(0.16, 2.2, 1.0, 0.9), hspace=0.12,
                          wspace=0.16)
    ax_r = fig.add_subplot(gs[0, 0])
    ax_v = fig.add_subplot(gs[1, 0], sharex=ax_r)
    ax_e = fig.add_subplot(gs[2, 0], sharex=ax_r)
    ax_c = fig.add_subplot(gs[3, 0], sharex=ax_r)
    ax_zv = fig.add_subplot(gs[0:2, 1])
    ax_ze = fig.add_subplot(gs[2, 1], sharex=ax_zv)
    ax_zt = fig.add_subplot(gs[3, 1])
    ax_zt.axis('off')

    m = sel(tv, t_a, t_b)
    me = sel(t_est, t_a, t_b)
    # regime strip (same regimes as the per-regime metrics)
    px_per_s = ax_r.get_position().width * fig.get_figwidth() * DPI / max(t_b - t_a, 1.0)
    for a_, b_, g in _regime_runs(tv[m], vr[m].astype(float)):
        ax_r.axvspan(a_ - T0, b_ - T0, color=REGIME_FILL[g], lw=0)
        ax_r.axvline(a_ - T0, color='white', lw=1.5)
        w_px = (b_ - a_) * px_per_s          # label only if it fits (~6.5 px per glyph at 7.5 pt)
        full, short = REGIME_SHORT[g]
        lab = full if w_px >= 6.5 * len(full) + 10 else (short if w_px >= 6.5 * len(short) + 6 else '')
        ax_r.text((a_ + b_) / 2 - T0, 0.5, lab, ha='center', va='center', fontsize=7.5, color=C_INK)
    ax_r.set_ylim(0, 1)
    ax_r.set_yticks([])
    ax_r.grid(False)
    for sp in ax_r.spines.values():
        sp.set_visible(False)
    ax_r.tick_params(bottom=False, labelbottom=False)
    ax_r.set_title('Скорость на окне %.0f с: режимы движения по эталону (как в метриках по режимам), эталон и оценка'
                   % (t_b - t_a), loc='left')

    ax_v.plot(tv[m] - T0, vr[m], color=C_REF, lw=3.0, label='эталон: GNSS RTK (master)', solid_capstyle='round')
    ax_v.plot(t_est[me] - T0, E['v'][o][me], color=C1, lw=1.2, label='оценка /result/velocity')
    ax_v.set_ylabel('скорость, м/с')
    ax_v.legend(loc='upper left', ncol=2)
    ymax = float(np.nanmax(vr[m])) * 1.15 + 0.5
    ax_v.set_ylim(-0.6, ymax)
    plt.setp(ax_v.get_xticklabels(), visible=False)

    err = ve[m]
    ax_e.axhline(0, color=C_INK, lw=0.6)
    ax_e.axhspan(-0.05, 0.05, color='#efeee9', lw=0, zorder=0)
    ax_e.plot(tv[m] - T0, err, color=C1, lw=0.9)
    lim = max(0.12, float(np.nanpercentile(np.abs(err), 99.5)) * 1.3) if np.isfinite(err).any() else 0.2
    ax_e.set_ylim(-lim, lim)
    ax_e.set_ylabel('ошибка, м/с')
    fin = np.isfinite(err)
    _note(ax_e, 'на окне: RMSE %.3f м/с, MAE %.3f м/с, смещение %+.3f м/с; серая полоса ±0.05 м/с' % (
        np.sqrt(np.mean(err[fin] ** 2)), np.mean(np.abs(err[fin])), np.mean(err[fin])), loc='lower left')
    plt.setp(ax_e.get_xticklabels(), visible=False)

    if 'cmd' in R:
        tc, pc = R['cmd']
        mc = sel(tc, t_a - 5, t_b)
        ax_c.step(tc[mc] - T0, pc[mc], where='post', color=C_INK, lw=1.0)
        ax_c.axhline(0, color='#c9c8c2', lw=0.7)
    ax_c.set_ylabel('позиция\nконтроллера')
    ax_c.set_ylim(-16, 16)
    ax_c.set_yticks([-15, -8, 0, 8, 15])
    ax_c.text(0.005, 0.95, 'driver_position_cmd: тяга > 0, тормоз < 0', transform=ax_c.transAxes, fontsize=7.5,
              color=C_MUTED, va='top')
    ax_c.set_xlabel('время от начала окна, с')
    ax_c.set_xlim(0, t_b - T0)

    # zoom: braking to a stop, with the raw bogie speeds
    mz = sel(tv, za, zb)
    mze = sel(t_est, za, zb)
    ax_zv.plot(tv[mz] - T0, vr[mz], color=C_REF, lw=3.0, label='эталон GNSS RTK', solid_capstyle='round')
    if 'diag_v_front' in E:
        ax_zv.plot(t_est[mze] - T0, E['diag_v_front'][o][mze], color=C2, lw=0.9, alpha=0.9,
                   label='передняя тележка (сырой датчик × K)')
        ax_zv.plot(t_est[mze] - T0, E['diag_v_rear'][o][mze], color=C3, lw=0.9, alpha=0.9,
                   label='задняя тележка (сырой датчик × K)')
    ax_zv.plot(t_est[mze] - T0, E['v'][o][mze], color=C1, lw=1.5, label='оценка')
    ax_zv.set_title('Увеличение: торможение до остановки', loc='left')
    ax_zv.set_ylabel('скорость, м/с')
    ax_zv.legend(loc='upper right', fontsize=7.5)
    ax_zv.set_ylim(bottom=-0.4, top=float(np.nanmax(vr[mz])) * 1.3 + 0.5)
    plt.setp(ax_zv.get_xticklabels(), visible=False)
    ez = ve[mz]
    ax_ze.axhline(0, color=C_INK, lw=0.6)
    ax_ze.axhspan(-0.05, 0.05, color='#efeee9', lw=0, zorder=0)
    ax_ze.plot(tv[mz] - T0, ez, color=C1, lw=1.0, marker='.', ms=2.5)
    zl = max(0.12, float(np.nanmax(np.abs(ez))) * 1.25) if np.isfinite(ez).any() else 0.2
    ax_ze.set_ylim(-zl, zl)
    ax_ze.set_ylabel('ошибка, м/с')
    ax_ze.set_xlabel('время от начала окна, с')
    ax_ze.set_xlim(za - T0, zb - T0)
    for ax in (ax_v, ax_e, ax_c):
        ax.axvspan(za - T0, zb - T0, color='#f3f2ed', zorder=-1, lw=0)
    ax_v.text((za + zb) / 2 - T0, 0.97, 'увеличено справа →', transform=ax_v.get_xaxis_transform(), ha='center',
              va='top', fontsize=7.5, color=C_MUTED)
    mt = R['metrics']
    ax_zt.text(0.0, 0.45, '\n'.join([
        'Весь заезд %s (%s):' % (R['bag'], R['dir']),
        '  RMSE скорости %.3f м/с, MAE %.3f м/с;' % (mt.get('v_rmse', np.nan), mt.get('v_mae', np.nan)),
        '  смещение: разгон %+.3f, торможение %+.3f,' % (mt.get('v_bias_accel', np.nan), mt.get('v_bias_brake', np.nan)),
        '  стоянка %+.3f, равномерно %+.3f м/с.' % (mt.get('v_bias_stopped', np.nan), mt.get('v_bias_cruise', np.nan)),
        'K — калибровочный масштаб колеса (км/ч → м/с).']), transform=ax_zt.transAxes, va='top', ha='left',
        fontsize=8.5, color=C_INK)
    fig.suptitle('Оценка скорости по колёсам и тяговой модели — пример заезда %s (%s)' % (R['bag'], R['dir']),
                 x=0.07, ha='left', y=0.96)
    info = dict(bag=R['bag'], window_s=[float(t_a - tv[0]), float(t_b - tv[0])],
                zoom_s=[float(za - tv[0]), float(zb - tv[0])],
                window_rmse=float(np.sqrt(np.mean(err[fin] ** 2))) if fin.any() else None,
                window_mae=float(np.mean(np.abs(err[fin]))) if fin.any() else None)
    return _save(fig, out_dir, 'speed_example'), info


# ---------------------------------------------------------------- 2. speed error
def fig_speed_error(RES, rows, out_dir, xmax=0.2):
    fig, axs = plt.subplots(1, 3, figsize=(15.5, 4.9), gridspec_kw=dict(width_ratios=(1.3, 1, 1), wspace=0.26))
    ax = axs[0]
    info = {}
    tail = []
    for d in ('T2S', 'S2T'):
        e = np.concatenate([np.abs(r['v_err'][np.isfinite(r['v_err'])]) for r in RES.values()
                            if r.get('dir') == d and 'v_err' in r]).astype(float)
        e = np.sort(e)
        p = np.arange(1, len(e) + 1) / len(e)
        q = {k: float(np.percentile(e, k)) for k in (50, 95, 99)}
        nb = sum(1 for r in RES.values() if r.get('dir') == d)
        info[d] = dict(n_bags=nb, n=int(len(e)), frac_gt_xmax=float(np.mean(e > xmax)),
                       **{'p%d' % k: v for k, v in q.items()})
        ax.plot(e, p, color=C_DIR[d], lw=1.8, label='%s: p50 %.3f · p95 %.3f · p99 %.3f' % (d, q[50], q[95], q[99]))
        tail.append('%s %.2f %%' % (d, 100 * np.mean(e > xmax)))
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, 1.005)
    for y in (0.5, 0.95, 0.99):
        ax.axhline(y, color='#c9c8c2', lw=0.8, ls='--', zorder=0)
        ax.text(xmax * 0.995, y - 0.004, '%g' % y, fontsize=7, color=C_MUTED, va='top', ha='right')
    ax.set_xlabel('|ошибка скорости|, м/с')
    ax.set_ylabel('доля точек эталона')
    ax.set_title('Распределение |v_est − v_ref| по всем точкам эталона', loc='left')
    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 0.3), fontsize=8, title='квантили, м/с', title_fontsize=8)
    ax.text(0.99, 0.2, '%d + %d заездов, %d точек\nдоля > %.1f м/с: %s' % (
        info['T2S']['n_bags'], info['S2T']['n_bags'], info['T2S']['n'] + info['S2T']['n'], xmax, ', '.join(tail)),
        transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5, color=C_MUTED)

    regs = ('accel', 'brake', 'stopped', 'cruise')
    reg_lab = ('разгон\na > 0.15', 'торможение\na < −0.15', 'стоянка\nv < 0.1', 'равномерно\n|a| ≤ 0.15')
    x = np.arange(len(regs))
    w = 0.36
    for k, (metric, title, ylab) in enumerate((
            ('v_rmse_', 'RMSE скорости по режимам движения', 'RMSE, м/с'),
            ('v_bias_', 'Смещение (оценка − эталон) по режимам', 'смещение, м/с'))):
        ax = axs[k + 1]
        for i, d in enumerate(('T2S', 'S2T')):
            vals = [np.array([_m(r, metric + g) for r in rows if r['metrics']['master']['dir'] == d]) for g in regs]
            vals = [v[np.isfinite(v)] for v in vals]
            med = np.array([np.median(v) for v in vals])
            lo = np.array([np.percentile(v, 25) for v in vals])
            hi = np.array([np.percentile(v, 75) for v in vals])
            xx = x + (i - 0.5) * w
            ax.bar(xx, med, w * 0.92, color=C_DIR[d], label=d, zorder=2)
            ax.errorbar(xx, med, yerr=[med - lo, hi - med], fmt='none', ecolor=C_INK, elinewidth=0.9, capsize=3,
                        zorder=3)
            for xi, mi, l_, h_ in zip(xx, med, lo, hi):
                if metric == 'v_bias_':
                    up = mi >= 0
                    ax.text(xi, (h_ if up else l_) + (0.0006 if up else -0.0006), '%+.4f' % mi, rotation=90,
                            ha='center', va='bottom' if up else 'top', fontsize=7, color=C_INK)
                else:
                    ax.text(xi, h_ + 0.0012, '%.3f' % mi, ha='center', va='bottom', fontsize=7, color=C_INK)
            info.setdefault('regimes', {})[d + '_' + metric.rstrip('_')] = dict(zip(regs, med.round(4).tolist()))
        ax.set_xticks(x)
        ax.set_xticklabels(reg_lab, fontsize=8)
        ax.set_ylabel(ylab)
        ax.set_title(title, loc='left')
        ax.axhline(0, color=C_INK, lw=0.7)
        ax.legend(loc='upper right')
        if metric == 'v_bias_':
            yl = max(abs(v) for v in ax.get_ylim())
            ax.set_ylim(-yl * 1.25, yl * 1.25)
        else:
            ax.set_ylim(0, ax.get_ylim()[1] * 1.1)
    fig.text(0.6, -0.06, 'столбик — медиана по заездам, усы — межквартильный размах (p25–p75) по заездам; '
             'режимы — по сглаженному ускорению a (м/с²) и скорости v (м/с) эталона', ha='center', fontsize=7.5, color=C_MUTED)
    fig.suptitle('Точность скорости на %d оценочных заездах (эталон GNSS RTK master)' % len(rows), x=0.07,
                 ha='left', y=1.02)
    return _save(fig, out_dir, 'speed_error'), info


# ---------------------------------------------------------------- 3. along-track
def _bin_mean(s, y, edges):
    idx = np.digitize(s, edges) - 1
    ok = (idx >= 0) & (idx < len(edges) - 1) & np.isfinite(y)
    n = np.bincount(idx[ok], minlength=len(edges) - 1).astype(float)
    sm = np.bincount(idx[ok], weights=y[ok], minlength=len(edges) - 1)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(n > 0, sm / n, np.nan)


def fig_along_track(RES, out_dir, ylim=25.0):
    fig, axs = plt.subplots(2, 1, figsize=(15.5, 8.6), sharey=True, gridspec_kw=dict(hspace=0.32))
    info = {}
    for ax, d in zip(axs, ('T2S', 'S2T')):
        B = [r for r in RES.values() if r.get('dir') == d and 's_ref' in r and len(r['s_ref'])]
        if not B:
            continue
        L = B[0]['L']
        fine = np.arange(0.0, L + 10.0, 10.0)
        coarse = np.arange(0.0, L + 25.0, 25.0)
        M = []
        n_clip = 0
        for r in B:
            s, a = r['s_ref'].astype(float), r['al_on'].astype(float)
            y = _bin_mean(s, a, fine)
            n_clip += int(np.nanmax(np.abs(y)) > ylim) if np.isfinite(y).any() else 0
            ax.plot((fine[:-1] + fine[1:]) / 2, np.clip(y, -ylim, ylim), color=C_DIR[d], lw=0.6, alpha=0.45,
                    zorder=2)
            M.append(_bin_mean(s, a, coarse))
        M = np.array(M)
        cnt = np.isfinite(M).sum(axis=0)
        ok = cnt >= 5
        xc = (coarse[:-1] + coarse[1:]) / 2
        with np.errstate(all='ignore'):
            med = np.where(ok, np.nanmedian(M, axis=0), np.nan)
            p10 = np.where(ok, np.nanpercentile(M, 10, axis=0), np.nan)
            p90 = np.where(ok, np.nanpercentile(M, 90, axis=0), np.nan)
            ab90 = np.where(ok, np.nanpercentile(np.abs(M), 90, axis=0), np.nan)
        ax.fill_between(xc, p10, p90, color=C_BAND, alpha=0.5, lw=0, label='p10–p90 по заездам', zorder=1)
        ax.plot(xc, med, color=C_INK, lw=1.8, label='медиана по заездам', zorder=4)
        ax.plot(xc, ab90, color=C_INK, lw=0.9, ls='--', label='p90 от |ошибки|', zorder=4)
        ax.plot([], [], color=C_DIR[d], lw=0.8, label='отдельные заезды (%d)' % len(B))
        ax.axhline(0, color=C_INK, lw=0.6)
        ax.set_xlim(0, L)
        ax.set_ylim(-ylim, ylim)
        ax.set_xlabel('положение на маршруте %s: дуговая координата эталона s, м (0 = начало pathgraph)' % d)
        ax.set_ylabel('ошибка вдоль пути, м\n(+ — оценка впереди)')
        allv = np.concatenate([r['al_on'] for r in B]).astype(float)
        allv = allv[np.isfinite(allv)]
        info[d] = dict(n_bags=len(B), pooled_abs_p50=float(np.median(np.abs(allv))),
                       pooled_abs_p90=float(np.percentile(np.abs(allv), 90)),
                       pooled_rmse=float(np.sqrt(np.mean(allv ** 2))), n_clipped_bags=n_clip,
                       band_p90_abs_max=float(np.nanmax(ab90)))
        ax.set_title('%s: ошибка вдоль пути на карте (L = %.0f м); по всем точкам |ошибка| p50 %.1f м, p90 %.1f м, '
                     'RMSE %.1f м' % (d, L, info[d]['pooled_abs_p50'], info[d]['pooled_abs_p90'],
                                      info[d]['pooled_rmse']), loc='left')
        ax.legend(loc='upper right', ncol=4, fontsize=7.5)
        if n_clip:
            ax.text(0.005, 0.03, 'значения за пределами ±%g м обрезаны (%d заезд.)' % (ylim, n_clip),
                    transform=ax.transAxes, fontsize=7, color=C_MUTED)
    fig.suptitle('Ошибка вдоль пути по дуговой координате маршрута (60 оценочных заездов, эталон RTK master)',
                 x=0.07, ha='left', y=0.965)
    return _save(fig, out_dir, 'along_track'), info


# ---------------------------------------------------------------- 4. trajectory
def _zoom_box(P, pad=30.0, min_half=60.0):
    P = P[np.isfinite(P).all(axis=1)]
    cx, cy = (P[:, 0].min() + P[:, 0].max()) / 2, (P[:, 1].min() + P[:, 1].max()) / 2
    half = max(np.ptp(P[:, 0]), np.ptp(P[:, 1])) / 2 + pad
    half = max(half, min_half)
    return cx - half, cx + half, cy - half, cy + half


def fig_trajectory(RD, out_dir, zoom_m=150.0):
    from tram_backup_odometry.core.track_map import load_routes
    from tram_backup_odometry.replay import default_maps_dir
    routes = load_routes(default_maps_dir(), 31.0)
    fig = plt.figure(figsize=(17.0, 10.4))
    gs = fig.add_gridspec(2, 4, height_ratios=(1.05, 1.0), hspace=0.2, wspace=0.22)
    ax_o = fig.add_subplot(gs[0, :])
    info = {}

    def draw_routes(ax, lw_scale=1.0):
        ax.plot(routes['T2S'].x, routes['T2S'].y, color=C_MAP, lw=6 * lw_scale, solid_capstyle='round', zorder=1)
        ax.plot(routes['S2T'].x, routes['S2T'].y, color='#bebdb6', lw=1.4 * lw_scale, zorder=1.5)

    draw_routes(ax_o)
    ax_o.plot([], [], color=C_MAP, lw=6, label='pathgraph T2S')
    ax_o.plot([], [], color='#bebdb6', lw=1.4, label='pathgraph S2T')
    # zoom panels in geographic order: west terminus (T2S start, S2T end), east terminus (T2S end, S2T start)
    panels = (('T2S', 'start'), ('S2T', 'end'), ('T2S', 'end'), ('S2T', 'start'))
    prep = {}
    for d in ('T2S', 'S2T'):
        R = RD.get(d)
        if R is None:
            continue
        E = R['est']
        tE = np.asarray(E['t'], float)
        o = np.argsort(tE, kind='stable')
        hp = np.asarray(E['has_pose'], bool)[o]
        prep[d] = dict(t=tE[o][hp], x=np.asarray(E['x'])[o][hp], y=np.asarray(E['y'])[o][hp],
                       s=np.asarray(E['diag_s_route'], float)[o][hp])
        Pr_all = R['series']['ref_xyz_all']
        ax_o.plot(Pr_all[::5, 0], Pr_all[::5, 1], '.', color=C_INK, ms=0.8, zorder=3)
        ax_o.plot(prep[d]['x'][::10], prep[d]['y'][::10], color=C_DIR[d], lw=1.1, zorder=4,
                  label='оценка %s (%s)' % (d, R['bag']))
        info[d] = dict(bag=R['bag'], L=float(routes[d].L))
    ax_o.plot([], [], '.', color=C_INK, ms=4, label='эталон RTK master')
    boxes = []
    for col, (d, part) in enumerate(panels):
        if d not in prep:
            continue
        R, Q = RD[d], prep[d]
        S = R['series']
        rt = routes[d]
        L = rt.L
        tp, Pr, Pe, e2d = S['tp'], S['xyz_ref'], S['xyz_est'], S['e2d']
        ax = fig.add_subplot(gs[1, col])
        m = Q['s'] <= min(zoom_m, 0.5 * L) if part == 'start' else Q['s'] >= L - zoom_m
        t_lo, t_hi = (Q['t'][m].min(), Q['t'][m].max()) if m.any() else (Q['t'][-1] - 60, Q['t'][-1])
        mp = (tp >= t_lo) & (tp <= t_hi)
        box = _zoom_box(np.r_[np.c_[Q['x'][m], Q['y'][m]], Pr[mp, :2]] if m.any() else Pr[mp, :2])
        draw_routes(ax, 1.6)
        ax.plot(Pr[mp, 0], Pr[mp, 1], color=C_INK, lw=0, marker='.', ms=2.2, zorder=3, label='эталон RTK master')
        ax.plot(Q['x'][m], Q['y'][m], color=C_DIR[d], lw=1.7, zorder=4, label='оценка /result/position')
        k = np.where(mp)[0]
        if len(k):                                   # error connectors every ~5 s
            kk = k[np.r_[True, np.diff(np.floor((tp[k] - tp[k[0]]) / 5.0)) > 0]]
            for i in kk:
                ax.plot([Pr[i, 0], Pe[i, 0]], [Pr[i, 1], Pe[i, 1]], color=C_MUTED, lw=0.6, zorder=2.5)
            ax.plot([], [], color=C_MUTED, lw=0.6, label='ошибка (отрезки через 5 с)')
        pe = (rt.x[0], rt.y[0]) if part == 'start' else (rt.x[-1], rt.y[-1])
        ax.plot(*pe, marker='X', ms=9, color=C_INK, mec='white', zorder=6, ls='none', label='начало / конец карты')
        if part == 'start':
            ax.plot(Q['x'][0], Q['y'][0], marker='o', ms=7, color=C_DIR[d], mec='white', zorder=6, ls='none',
                    label='первая поза (после выставки)')
        ax.set_xlim(box[0], box[1])
        ax.set_ylim(box[2], box[3])
        ax.set_aspect('equal', adjustable='box')
        lt = 'ABCD'[col]
        if mp.any():
            e_in = e2d[mp]
            txt = '2D-ошибка: средн. %.1f м, макс. %.1f м' % (e_in.mean(), e_in.max())
            if part == 'end':
                txt += '\nв конце записи: %.1f м' % e2d[-1]
            _note(ax, txt, loc='lower right' if col in (0, 1) else 'lower left', fs=7.5)
            info['%s_%s' % (d, part)] = dict(e2d_mean=float(e_in.mean()), e2d_max=float(e_in.max()),
                                             e2d_last=float(e2d[-1]) if part == 'end' else None)
        ttl = {'start': 'начало: мост G1 к карте', 'end': 'конец: хвост за картой'}[part]
        ax.set_title('%s  %s — %s' % (lt, d, ttl), loc='left', fontsize=9.5, color=C_INK)
        ax.tick_params(labelsize=7)
        ax.set_xlabel('x, м', fontsize=8)
        if col == 0:
            ax.set_ylabel('y, м', fontsize=8)
        boxes.append(box)
        ax_o.add_patch(Rectangle((box[0], box[2]), box[1] - box[0], box[3] - box[2], fill=False, ec=C_DIR[d],
                                 lw=1.3, zorder=5))
        west = col in (0, 1)
        ax_o.text(box[0] - 30 if west else box[1] + 30, box[3] if d == 'T2S' else box[2], lt, fontsize=11,
                  fontweight='bold', color=C_DIR[d], ha='right' if west else 'left',
                  va='top' if d == 'T2S' else 'bottom', zorder=6)
        if col == 3:
            ax.legend(loc='upper left', fontsize=7, bbox_to_anchor=(1.02, 1.0), borderaxespad=0)
    xa = np.concatenate([routes[k].x for k in routes] + [np.r_[b[0], b[1]] for b in boxes])
    ya = np.concatenate([routes[k].y for k in routes] + [np.r_[b[2], b[3]] for b in boxes])
    ax_o.set_xlim(xa.min() - 170, xa.max() + 170)       # room for the A-D letters beside the boxes
    ax_o.set_ylim(ya.min() - 40, ya.max() + 40)
    ax_o.set_aspect('equal', adjustable='box', anchor='C')
    ax_o.set_xlabel('x, м (UTM 37N − 300000)')
    ax_o.set_ylabel('y, м (UTM 37N − 6100000)')
    ax_o.set_title('Маршрут в кадре map (pathgraph); рамки A–D — области увеличения ниже', loc='left')
    ax_o.legend(loc='upper left', fontsize=7.5, ncol=1)
    fig.suptitle('Траектории: эталон RTK и оценка для заездов T2S (%s) и S2T (%s)' % (
        RD['T2S']['bag'] if 'T2S' in RD else '-', RD['S2T']['bag'] if 'S2T' in RD else '-'), x=0.07, ha='left',
        y=0.94)
    return _save(fig, out_dir, 'trajectory'), info


# ---------------------------------------------------------------- 5. per bag
def fig_per_bag(rows, out_dir, tag):
    keys = (('al_rmse', 'RMSE ошибки\nвдоль пути, м', 22.0),
            ('e3d_mean', 'средняя\n3D-ошибка, м', 22.0),
            ('drift_pct', 'дрейф в конце,\n% пройденного пути', 5.5))
    order = []
    for d in ('T2S', 'S2T'):
        R = [r for r in rows if r['metrics']['master']['dir'] == d]
        order += sorted(R, key=lambda r: _m(r, 'e3d_mean'))
    n = len(order)
    x = np.arange(n)
    fig, axs = plt.subplots(3, 1, figsize=(15.5, 10.5), sharex=True, gridspec_kw=dict(hspace=0.14))
    cols = [C_DIR[r['metrics']['master']['dir']] for r in order]
    n_t2s = sum(1 for r in order if r['metrics']['master']['dir'] == 'T2S')
    for ax, (k, lab, cap) in zip(axs, keys):
        v = np.array([_m(r, k) for r in order])
        ax.bar(x, np.minimum(v, cap), 0.8, color=cols, zorder=2)
        for i in np.where(v > cap)[0]:
            ax.text(x[i], cap * 0.98, '%.0f↑' % v[i], ha='center', va='top', fontsize=7, color='white',
                    fontweight='bold', zorder=3, rotation=90)
        for d, sl in (('T2S', slice(0, n_t2s)), ('S2T', slice(n_t2s, n))):
            med = np.nanmedian(v[sl])
            ax.hlines(med, x[sl][0] - 0.4, x[sl][-1] + 0.4, color=C_INK, lw=1.2, ls='--', zorder=4)
            ax.text((x[sl][0] + x[sl][-1]) / 2, cap * 0.96, '%s: медиана %.2f (пунктир)' % (d, med), fontsize=8,
                    va='top', ha='center', color=C_INK, zorder=5,
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='#d0cfc9', alpha=0.9))
        ax.axvline(n_t2s - 0.5, color='#bebdb6', lw=1.0)
        ax.set_ylim(0, cap)
        ax.set_ylabel(lab)
        ax.grid(axis='x', visible=False)
    axs[0].legend(handles=[Patch(color=C1, label='T2S (%d)' % n_t2s), Patch(color=C2, label='S2T (%d)' % (n - n_t2s))],
                  loc='upper left', ncol=2)
    axs[-1].set_xticks(x)
    axs[-1].set_xticklabels([r['bag'] for r in order], rotation=90, fontsize=6.5)
    axs[-1].set_xlim(-0.7, n - 0.3)
    axs[-1].set_xlabel('заезд (внутри направления — по возрастанию средней 3D-ошибки)')
    fig.suptitle('Метрики положения по заездам (%s, эталон RTK master, %d оценочных заездов)' % (tag, n),
                 x=0.07, ha='left', y=0.93)
    return _save(fig, out_dir, 'per_bag'), {}


# ---------------------------------------------------------------- 6. ablations
def fig_ablations(out_dir, tags=AF_ORDER, far=('B1_naive', 'AF_noloops', 'G_limit0')):
    S = {}
    for t in tags:
        p = os.path.join(RESULTS, t, 'summary.json')
        if os.path.exists(p):
            with open(p) as f:
                S[t] = json.load(f)['overall']
    tags = [t for t in tags if t in S]
    metrics = (('al_rmse', 'RMSE вдоль пути, м'), ('e3d_mean', 'средняя 3D-ошибка, м'),
               ('drift_pct', 'дрейф в конце, % пути'), ('score_loss', 'сводный score_loss'))
    ny = len(tags)
    y = np.arange(ny)[::-1].astype(float)
    gi = [i for i, t in enumerate(tags) if t.startswith('G_')]
    if gi:                                      # visual gap before the GNSS-window group
        y[gi[0]:] -= 0.7
    fig, axs = plt.subplots(1, 4, figsize=(16.5, 0.4 * ny + 1.6), sharey=True, gridspec_kw=dict(wspace=0.08))
    info = {}
    for ax, (k, lab) in zip(axs, metrics):
        med = np.array([S[t][k]['median'] for t in tags])
        mean = np.array([S[t][k]['mean'] for t in tags])
        ref_med = S['F4_final'][k]['median'] if 'F4_final' in S else None
        near = [i for i, t in enumerate(tags) if t not in far] or list(range(ny))
        cap = max(np.max(med[near]), np.max(mean[near])) * 1.6
        for i, t in enumerate(tags):
            col = C1 if t == 'F4_final' else ('#6b6a65' if t == 'B1_naive' else ('#c7c6bf' if t.startswith('G_')
                                                                                 else '#a9a8a1'))
            ax.barh(y[i], min(med[i], cap), 0.7, color=col, zorder=2)
            if mean[i] <= cap * 0.97:
                ax.plot(mean[i], y[i], marker='D', ms=5, mfc='white', mec=C_INK, mew=1.0, zorder=3, ls='none')
            if max(med[i], mean[i]) + 0.14 * cap <= cap:          # room for the label after the marks
                ax.text(max(med[i], mean[i]) + cap * 0.025, y[i], '%.3g' % med[i], va='center',
                        fontsize=7.5, color=C_INK, fontweight='bold' if t == 'F4_final' else 'normal')
            else:                                                  # label inside the (possibly clipped) bar
                ax.text(min(med[i], cap) - cap * 0.015, y[i], '%.3g (ср. %.3g)%s' % (
                    med[i], mean[i], ' →' if med[i] > cap else ''), ha='right', va='center', fontsize=7.5,
                    color='white' if col in (C1, '#6b6a65') else C_INK, fontweight='bold', zorder=4)
        if ref_med is not None:
            ax.axvline(ref_med, color=C1, lw=1.0, ls='--', zorder=1)
        ax.set_xlim(0, cap)
        ax.set_title(lab, loc='left', fontsize=9.5)
        ax.grid(axis='y', visible=False)
        info[k] = {t: dict(median=float(med[i]), mean=float(mean[i])) for i, t in enumerate(tags)}
    axs[0].set_yticks(y)
    axs[0].set_yticklabels([AF_LABELS.get(t, t) for t in tags])
    axs[0].set_ylim(y.min() - 0.6, y.max() + 0.6)
    handles = [Patch(color=C1, label='медиана: итоговая версия'), Patch(color='#a9a8a1', label='медиана: вариант'),
               plt.Line2D([], [], marker='D', ms=5, mfc='white', mec=C_INK, ls='none', label='среднее по заездам')]
    fig.subplots_adjust(bottom=0.13)
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.55, 0.075), ncol=3, fontsize=8, frameon=False)
    fig.text(0.55, 0.012, 'синий пунктир — медиана итоговой версии; «→» — столбик обрезан по оси', ha='center',
             fontsize=7.5, color=C_MUTED)
    fig.suptitle('Абляции и окно GNSS: 60 оценочных заездов, эталон RTK master (меньше — лучше; число — медиана)',
                 x=0.07, ha='left', y=0.99)
    return _save(fig, out_dir, 'ablations'), info


# ---------------------------------------------------------------- 7. real time
def _read_csv(path):
    with open(path) as f:
        r = csv.DictReader(f)
        rows = list(r)
    return rows


def _latency(path, warmup=2.0):
    rows = _read_csv(path)
    out = {}
    for kind in ('velocity', 'position'):
        w = np.array([float(r['wall_s']) for r in rows if r['output'] == kind])
        lat = np.array([float(r['latency_ms']) for r in rows if r['output'] == kind])
        if len(w):
            out[kind] = dict(w=w, lat=lat, steady=w >= w[0] + warmup)
    return out


def _resources(path):
    rows = _read_csv(path)
    return {k: np.array([float(r[k]) for r in rows]) for k in ('t_s', 'cpu_cores', 'rss_mib')}


def _rolling(y, n):
    if len(y) < n or n < 2:
        return y
    c = np.cumsum(np.r_[0.0, y])
    r = (c[n:] - c[:-n]) / n
    return np.r_[np.full(n // 2, np.nan), r, np.full(len(y) - len(r) - n // 2, np.nan)]


def fig_realtime(out_dir, short_dir, full_dir):
    full_res = os.path.join(full_dir, 'resources.csv') if full_dir else None
    have_full = bool(full_res and os.path.exists(full_res) and os.path.getsize(full_res) > 50)
    full_lat = os.path.join(full_dir, 'latency.csv') if full_dir else None
    have_full_lat = bool(full_lat and os.path.exists(full_lat) and os.path.getsize(full_lat) > 50)
    nrow = 3 if have_full else 2
    fig, axs = plt.subplots(nrow, 2, figsize=(15.5, 4.0 * nrow), gridspec_kw=dict(hspace=0.42, wspace=0.18))
    Ls = _latency(os.path.join(short_dir, 'latency.csv'))
    info = {'short': {}, 'full': None}
    name_s = os.path.basename(short_dir.rstrip('/'))
    # (a) CDF
    ax = axs[0, 0]
    styles = {'velocity': (C1, '/result/velocity'), 'position': (C2, '/result/position')}

    def stats(dct):
        lat, st = dct['lat'], dct['lat'][dct['steady']]
        return dict(p50=float(np.percentile(lat, 50)), p99=float(np.percentile(lat, 99)), max=float(lat.max()),
                    steady_p99=float(np.percentile(st, 99)), steady_max=float(st.max()), n=int(len(lat)))

    runs = [('120 с', Ls, '-', 'short')]
    if have_full_lat:
        Lf = _latency(full_lat)
        dur = max(d['w'][-1] - d['w'][0] for d in Lf.values())
        runs.append(('%.0f мин' % (dur / 60), Lf, '--', 'full'))
        info['full'] = {}
    rows_txt = ['мс: p50 / p99 / макс (после 2 с: p99 / макс)']
    for rl, L_, ls_, key in runs:
        for kind, dct in L_.items():
            col, lab = styles[kind]
            lat = np.sort(dct['lat'])
            ax.plot(lat, np.arange(1, len(lat) + 1) / len(lat), color=col, lw=1.8 if ls_ == '-' else 1.4, ls=ls_,
                    label='%s — %s' % (lab, rl))
            st_ = stats(dct)
            info[key][kind] = st_
            rows_txt.append('%s %s: %.1f / %.1f / %.1f (%.1f / %.1f)' % (
                kind, rl, st_['p50'], st_['p99'], st_['max'], st_['steady_p99'], st_['steady_max']))
    ax.text(0.37, 0.47, '\n'.join(rows_txt), transform=ax.transAxes, fontsize=7, color=C_INK, va='bottom',
            ha='left', linespacing=1.5, bbox=dict(boxstyle='round,pad=0.35', fc='white', ec='#d6d5cf', alpha=0.95))
    for xv, lab in ((100, 'норма 100 мс'), (250, 'пик 250 мс')):
        ax.axvline(xv, color='#c0392b' if xv == 250 else '#e08a3c', lw=1.2, ls=':')
        ax.text(xv * 0.93, 0.44, lab, rotation=90, fontsize=7.5, ha='right', va='bottom', color=C_INK)
    ax.set_xscale('log')
    ax.set_xlim(0.1, 400)
    ax.set_ylim(0, 1.005)
    ax.set_xlabel('задержка «вход → выход», мс (лог. шкала)')
    ax.set_ylabel('доля сообщений')
    ax.set_title('Задержка публикации: CDF (ros2 bag play -r 1)', loc='left')
    ax.legend(loc='lower right', fontsize=7.5)
    # (b) timeline
    ax = axs[0, 1]
    w0 = min(d['w'][0] for d in Ls.values())
    for kind, dct in Ls.items():
        ax.plot(dct['w'] - w0, dct['lat'], '.', ms=1.8, color=styles[kind][0], label=styles[kind][1], alpha=0.8)
    ax.axvspan(0, 2.0, color='#f3e3d6', lw=0, zorder=0)
    ax.text(2.2, 60, 'стартовый всплеск\n(очередь при запуске воспроизведения)', fontsize=7.5, color=C_INK, va='center')
    for yv in (100, 250):
        ax.axhline(yv, color='#c0392b' if yv == 250 else '#e08a3c', lw=1.2, ls=':')
    ax.set_yscale('log')
    ax.set_ylim(0.1, 400)
    ax.set_xlabel('время от первого выхода, с')
    ax.set_ylabel('задержка, мс (лог.)')
    ax.set_title('Задержка во времени — прогон 120 с (%s)' % name_s, loc='left')
    ax.legend(loc='upper right', markerscale=5)
    # (c, d) resources of the short run
    Rs = _resources(os.path.join(short_dir, 'resources.csv'))
    info['short']['cpu_mean'] = float(Rs['cpu_cores'].mean())
    info['short']['cpu_max'] = float(Rs['cpu_cores'].max())
    info['short']['rss_max'] = float(Rs['rss_mib'].max())
    _res_panels(axs[1, 0], axs[1, 1], Rs, 'прогон 120 с (%s)' % name_s)
    if have_full:
        Rf = _resources(full_res)
        _res_panels(axs[2, 0], axs[2, 1], Rf, 'полный заезд %s (%.0f мин)' % (
            os.path.basename(full_dir.rstrip('/')), Rf['t_s'][-1] / 60), minutes=True)
        info['full_resources'] = dict(cpu_mean=float(Rf['cpu_cores'].mean()), cpu_max=float(Rf['cpu_cores'].max()),
                                      rss_start=float(Rf['rss_mib'][0]), rss_max=float(Rf['rss_mib'].max()),
                                      duration_s=float(Rf['t_s'][-1]))
    fig.suptitle('Реальное время: узел в Docker ros:humble, воспроизведение bag в реальном темпе', x=0.07,
                 ha='left', y=0.985 if nrow == 2 else 0.955)
    return _save(fig, out_dir, 'realtime'), info


def _res_panels(ax_c, ax_m, R, what, minutes=False):
    t = R['t_s'] / (60.0 if minutes else 1.0)
    tu = 'мин' if minutes else 'с'
    cpu = R['cpu_cores']
    ax_c.plot(t, cpu, color=C1, lw=0.7 if minutes else 1.0, alpha=0.6 if minutes else 1.0, label='интервал 0.5 с')
    if minutes:
        ax_c.plot(t, _rolling(cpu, 60), color=C_INK, lw=1.4, label='скользящее среднее 30 с')
    ax_c.axhline(cpu.mean(), color=C_INK, lw=1.0, ls='--')
    ax_c.set_ylim(0, max(0.3, cpu.max() * 1.25))
    ax_c.set_xlim(0, t[-1])
    ax_c.set_xlabel('время, %s' % tu)
    ax_c.set_ylabel('CPU, ядер')
    ax_c.set_title('CPU узла, ядер — %s' % what, loc='left', fontsize=9.5)
    _note(ax_c, 'лимит 2 ядра; макс %.2f, среднее %.3f' % (cpu.max(), cpu.mean()), loc='upper left')
    ax_c.plot([], [], color=C_INK, lw=1.0, ls='--', label='среднее')
    ax_c.legend(loc='upper right')
    rss = R['rss_mib']
    ax_m.plot(t, rss, color=C2, lw=1.4)
    ax_m.set_ylim(0, max(100.0, rss.max() * 1.3))
    ax_m.set_xlim(0, t[-1])
    ax_m.set_xlabel('время, %s' % tu)
    ax_m.set_ylabel('RSS, МиБ')
    ax_m.set_title('Память (RSS) — %s' % what, loc='left', fontsize=9.5)
    _note(ax_m, 'лимит 512 МиБ', loc='upper left')
    ax_m.text(t[-1], rss.max(), 'старт %.1f → макс %.1f МиБ ' % (rss[0], rss.max()), ha='right', va='bottom',
              fontsize=7.5, color=C_INK)


# ---------------------------------------------------------------- 8. covariance
def fig_covariance(R, rows, out_dir):
    C = R['cov']
    tp = C['tp']
    t = (tp - tp[0]) / 60.0
    fig = plt.figure(figsize=(15.5, 8.2))
    gs = fig.add_gridspec(2, 2, width_ratios=(2.2, 1.0), hspace=0.3, wspace=0.2)
    ax_a = fig.add_subplot(gs[0, 0])
    ax_c = fig.add_subplot(gs[1, 0], sharex=ax_a)
    ax_p = fig.add_subplot(gs[:, 1])
    off = np.asarray(C['off'], float) > 0
    info = {}
    for ax, e, var, lab in ((ax_a, C['ea'], C['va'], 'вдоль пути'), (ax_c, C['ec'], C['vc'], 'поперёк пути')):
        sig = np.sqrt(np.maximum(var, 0))
        ax.fill_between(t, -2 * sig, 2 * sig, color='#cfe0f5', lw=0, label='±2σ из ковариации Odometry', zorder=1)
        ax.plot(t, e, color=C_INK, lw=0.9, label='фактическая ошибка (оценка − эталон)', zorder=3)
        # off-map spans
        if off.any():
            edges = np.flatnonzero(np.diff(np.r_[0, off.astype(int), 0]))
            for a, b in zip(edges[::2], edges[1::2]):
                ax.axvspan(t[a], t[b - 1], color='#f3e3d6', lw=0, zorder=0)
            ax.axvspan(0, 0, color='#f3e3d6', label='вне pathgraph (мост / хвост)')
        ax.set_yscale('symlog', linthresh=5.0, linscale=1.3)
        yl = max(np.nanmax(np.abs(e)), np.nanmax(2 * sig))
        ax.set_ylim(-yl * 1.5, yl * 1.5)
        tk = [v for v in ((2, 5, 20, 50) if yl < 100 else (2, 5, 20, 200, 1000)) if v < yl * 1.5]
        ax.set_yticks([-v for v in tk[::-1]] + [0] + tk)
        ax.set_yticklabels(['%g' % -v for v in tk[::-1]] + ['0'] + ['%g' % v for v in tk])
        ax.yaxis.set_minor_locator(matplotlib.ticker.NullLocator())
        ax.axhline(0, color=C_INK, lw=0.5)
        ax.set_ylabel('ошибка %s, м\n(симметр. лог.: линейно в ±5 м)' % lab)
        inside = np.abs(e) <= 2 * sig
        info[lab] = dict(inside_2sigma=float(np.mean(inside)), onmap_inside=float(np.mean(inside[~off])) if (~off).any()
                         else None)
        _note(ax, 'в пределах ±2σ: %.0f %% точек (на карте %.0f %%)' % (
            100 * inside.mean(), 100 * np.mean(inside[~off]) if (~off).any() else np.nan), loc='lower left')
        ax.set_title('Ошибка %s и заявленная неопределённость — заезд %s (%s)' % (lab, R['bag'], R['dir']),
                     loc='left', fontsize=9.5)
    ax_a.legend(loc='upper center', ncol=3, fontsize=7.5)
    plt.setp(ax_a.get_xticklabels(), visible=False)
    ax_c.set_xlabel('время от первого эталонного фикса, мин')
    # per-bag p_in95
    vals = sorted(((_m(r, 'p_in95'), r['metrics']['master']['dir'], r['bag']) for r in rows
                   if np.isfinite(_m(r, 'p_in95'))), key=lambda v: v[0])
    y = np.arange(len(vals))
    ax_p.scatter([v[0] for v in vals], y, c=[C_DIR[v[1]] for v in vals], s=22, zorder=3, edgecolors='white',
                 linewidths=0.5)
    ax_p.axvline(0.95, color=C_INK, lw=1.2, ls='--')
    ax_p.text(0.948, len(vals) * 0.02, 'цель 0.95 ', ha='right', fontsize=8, color=C_INK)
    pv = np.array([v[0] for v in vals])
    for i, v in enumerate(vals):
        if v[2] == R['bag']:
            ax_p.annotate(v[2], (v[0], i), xytext=(-60, 0), textcoords='offset points', fontsize=7, va='center',
                          arrowprops=dict(arrowstyle='-', color=C_MUTED, lw=0.6))
        elif v[0] < 0.6:
            ax_p.text(v[0] + 0.01, i, v[2], fontsize=6.5, va='center', color=C_MUTED)
    ax_p.set_yticks([])
    ax_p.set_xlim(max(0.0, pv.min() - 0.08), 1.02)
    ax_p.set_ylim(-1, len(vals))
    ax_p.set_xlabel('доля точек внутри 95 %-эллипса (p_in95)')
    ax_p.set_ylabel('заезды, по возрастанию')
    ax_p.scatter([], [], c=C1, s=22, label='T2S')
    ax_p.scatter([], [], c=C2, s=22, label='S2T')
    ax_p.legend(loc='center left')
    ax_p.set_title('Согласованность ковариации по заездам', loc='left', fontsize=9.5)
    _note(ax_p, 'медиана %.3f\n≥ 0.90: %d из %d заездов\n< 0.70: %d' % (
        np.median(pv), int(np.sum(pv >= 0.9)), len(pv), int(np.sum(pv < 0.7))), loc='upper left')
    info['p_in95_median'] = float(np.median(pv))
    fig.suptitle('Ковариация положения в /result/position: ±2σ против фактической ошибки', x=0.07, ha='left',
                 y=0.965)
    return _save(fig, out_dir, 'covariance'), info


# ---------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'plots'))
    ap.add_argument('--tag', default='F4_final', help='results/<tag>/ with per_bag.json of the same code')
    ap.add_argument('--only', default='all', help='comma list of: ' + ', '.join(FIGS))
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    ap.add_argument('--t2s-bag')
    ap.add_argument('--s2t-bag')
    ap.add_argument('--speed-bag', help='default: the T2S representative bag')
    ap.add_argument('--speed-t0', type=float, help='window start, s after the first reference sample (150 s window)')
    ap.add_argument('--cov-bag', help='default: the S2T representative bag')
    ap.add_argument('--rt-short', default=os.path.join(RESULTS, 'rt', '0e41eac3_120s_final'))
    ap.add_argument('--rt-full', default=os.path.join(RESULTS, 'rt', '27e994fc_full_final'))
    ap.add_argument('--cache', help='directory to keep/reuse the replay results (development only)')
    a = ap.parse_args(argv)
    figs = FIGS if a.only == 'all' else tuple(f.strip() for f in a.only.split(',') if f.strip())
    bad = [f for f in figs if f not in FIGS]
    if bad:
        ap.error('unknown figure(s): %s' % ', '.join(bad))
    os.makedirs(a.out, exist_ok=True)
    rows = load_per_bag(a.tag)
    t2s = a.t2s_bag or pick_representative(rows, 'T2S')
    s2t = a.s2t_bag or pick_representative(rows, 'S2T')
    speed_bag = a.speed_bag or t2s
    cov_bag = a.cov_bag or s2t
    print('representative bags: T2S %s, S2T %s; speed %s; covariance %s' % (t2s, s2t, speed_bag, cov_bag))
    detail = set()
    if 'speed_example' in figs:
        detail.add(speed_bag)
    if 'trajectory' in figs:
        detail |= {t2s, s2t}
    if 'covariance' in figs:
        detail.add(cov_bag)
    bags = sorted(detail)
    if NEED_ALL & set(figs):
        bags = sorted(set(r['bag'] for r in rows) | detail)
    RES = run_replays(bags, detail, a.jobs, a.gnss_limit, a.cache) if bags else {}
    meta = dict(tag=a.tag, t2s_bag=t2s, s2t_bag=s2t, speed_bag=speed_bag, cov_bag=cov_bag, figures={})
    t0 = time.time()
    for f in figs:
        if f == 'speed_example':
            p, info = fig_speed_example(RES[speed_bag], a.out, a.speed_t0)
        elif f == 'speed_error':
            p, info = fig_speed_error({b: r for b, r in RES.items() if b in {x['bag'] for x in rows}}, rows, a.out)
        elif f == 'along_track':
            p, info = fig_along_track({b: r for b, r in RES.items() if b in {x['bag'] for x in rows}}, a.out)
        elif f == 'trajectory':
            p, info = fig_trajectory({'T2S': RES[t2s], 'S2T': RES[s2t]}, a.out)
        elif f == 'per_bag':
            p, info = fig_per_bag(rows, a.out, a.tag)
        elif f == 'ablations':
            p, info = fig_ablations(a.out)
        elif f == 'realtime':
            p, info = fig_realtime(a.out, a.rt_short, a.rt_full)
        elif f == 'covariance':
            p, info = fig_covariance(RES[cov_bag], rows, a.out)
        meta['figures'][f] = dict(file=os.path.relpath(p, ROOT), **({'info': info} if info else {}))
    mp = os.path.join(a.out, 'plots_meta.json')
    if os.path.exists(mp):                      # keep the entries of figures not redrawn this time
        try:
            with open(mp) as fh:
                old = json.load(fh)
            meta['figures'] = dict(old.get('figures', {}), **meta['figures'])
        except (OSError, ValueError):
            pass
    with open(mp, 'w') as fh:
        json.dump(ev._jsonable(meta), fh, indent=1, ensure_ascii=False)
    print('figures: %d in %.0f s -> %s' % (len(figs), time.time() - t0, os.path.relpath(a.out, ROOT)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
