#!/usr/bin/env python3
"""Parallel replay + evaluation over bag sets -> results/<tag>/{per_bag.json, summary.json, summary.md}.

Every selected bag is replayed through the estimator (default
``tram_backup_odometry.replay:run_bag``) from its parsed cache
``.work/cache/<bag>.pkl``; bags with an RTK reference are scored with
``tools/evaluate.py``.  Bags without GNSS are still replayed to prove the
estimator does not crash, publishes finite values and keeps the input stamps.

Parameters: ``Params`` defaults <- ``--params`` yaml <- ``--fold-params DIR/fold_<k>.yaml``
(the bag's leave-one-group-out fold, so offline calibrations can be scored out
of fold) <- ``--set key=value``.  Keys unknown to ``Params`` are set as extra
attributes (read by modules through ``getattr(params, name, default)``) and are
listed in the summary.  ``vehicle_id`` is taken from the bag prefix unless set.

GNSS: by default fixes later than ``--gnss-limit`` (10 s) after the first fix
are not fed to the estimator at all, as in the hidden test runs.

Aggregates use the ``eval_ok`` bags of ``.work/bag_index.json`` (unique, >= 60 s
of RTK reference); per direction, per vehicle and per group (= fold).

Timing: ``cpu_us_per_msg`` = replay CPU time / input messages; with the default
estimator every ``Pipeline.on_*`` call is also timed (``lat_ms_p99/max``).

    python tools/run_cv.py --tag B0_naive --bags all --jobs 8
    python tools/run_cv.py --tag trn_off --bags eval --set trn_enable=false --plots worst
    python tools/run_cv.py --compare B0_naive trn_off
"""
import argparse
import importlib
import importlib.util
import inspect
import json
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import yaml

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import evaluate as ev  # noqa: E402
import bag_index as bi  # noqa: E402

sys.path.insert(0, ev.PKG_DIR)
RESULTS = os.path.join(ev.ROOT, 'results')
DEFAULT_ESTIMATOR = 'tram_backup_odometry.replay:run_bag'
INPUT_TOPICS = ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd')
TABLE_KEYS = ('score_loss', 'v_rmse', 'v_mae', 'v_bias', 'v_bias_trans', 'v_p99', 'v_cov', 'p_cov',
              'e2d_mean', 'e3d_mean', 'e3d_max', 'al_mean', 'al_mean_abs', 'al_rmse', 'al_max', 'ct_rmse',
              'e2d_offmap_mean', 'drift_pct', 'drift_onmap_pct', 'al_growth_pct', 'p_in95', 'p_nees')
GROUP_KEYS = ('score_loss', 'v_rmse', 'v_bias_trans', 'al_mean_abs', 'al_rmse', 'al_max', 'e3d_mean', 'drift_pct')
WORST_BY = ('score_loss', 'al_rmse', 'v_rmse', 'drift_pct', 'e2d_offmap_mean')


# ------------------------------------------------------------------ params
def _read_yaml(path):
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    if len(d) == 1:
        only = next(iter(d.values()))
        if isinstance(only, dict) and 'ros__parameters' in only:
            d = only['ros__parameters']
    return d


def build_params(bag, fold, cfg):
    """Params for one bag + list of keys unknown to Params (set as extra attributes)."""
    from dataclasses import fields
    from tram_backup_odometry.core.config import Params
    d = {}
    if cfg.get('params'):
        d.update(_read_yaml(cfg['params']))
    if cfg.get('fold_params'):
        fp = os.path.join(cfg['fold_params'], 'fold_%d.yaml' % fold)
        if os.path.exists(fp):
            d.update(_read_yaml(fp))
    # the CV reference is the GNSS master antenna: publish that point (the package
    # default is base_link for the organisers' checker) unless --set overrides it
    d.update(output_along_offset_m=0.0, output_z_offset_m=Params().z_offset_m,
             output_velocity_delay_s=0.0)       # GNSS speed has no lag vs the wheels
    d.update(cfg.get('overrides') or {})
    known = {f.name for f in fields(Params)}
    p = Params.from_dict({k: v for k, v in d.items() if k in known})
    extra = sorted(k for k in d if k not in known)
    for k in extra:
        v = d[k]
        setattr(p, k, yaml.safe_load(v) if isinstance(v, str) else v)
    if not p.vehicle_id and bag[:5] in ('30618', '30639'):
        p.vehicle_id = bag[:5]
    return p, extra


def parse_sets(items):
    out = {}
    for kv in items or []:
        if '=' not in kv:
            raise SystemExit('--set expects key=value, got %r' % kv)
        k, v = kv.split('=', 1)
        out[k.strip()] = v.strip()
    return out


# ------------------------------------------------------------------ estimator
def load_estimator(spec):
    """'package.module:function' or '/path/to/file.py:function'."""
    mod_name, fn_name = spec.rsplit(':', 1)
    if mod_name.endswith('.py'):
        name = '_est_' + os.path.splitext(os.path.basename(mod_name))[0]
        if name not in sys.modules:
            spec_ = importlib.util.spec_from_file_location(name, os.path.abspath(mod_name))
            mod = importlib.util.module_from_spec(spec_)
            spec_.loader.exec_module(mod)
            sys.modules[name] = mod
        mod = sys.modules[name]
    else:
        mod = importlib.import_module(mod_name)
    return mod, getattr(mod, fn_name)


def _timed_pipeline(base, sink):
    """Subclass of the pipeline that records the wall time of every on_* call."""
    import time as _t

    class Timed(base):
        def on_wheel(self, *a, **k):
            t0 = _t.perf_counter(); r = base.on_wheel(self, *a, **k); sink.append(_t.perf_counter() - t0); return r

        def on_cmd(self, *a, **k):
            t0 = _t.perf_counter(); r = base.on_cmd(self, *a, **k); sink.append(_t.perf_counter() - t0); return r

        def on_fix(self, *a, **k):
            t0 = _t.perf_counter(); r = base.on_fix(self, *a, **k); sink.append(_t.perf_counter() - t0); return r
    return Timed


def run_estimator(spec, bag, params, D, gnss_limit, gnss_burst=None):
    """-> (estimate dict, cpu_s, wall_s, latencies array or None)."""
    mod, fn = load_estimator(spec)
    kw = {'data': D}
    sig = inspect.signature(fn).parameters
    if 'gnss_limit_s' in sig:
        kw['gnss_limit_s'] = gnss_limit
    if gnss_burst and 'gnss_burst' in sig:
        kw['gnss_burst'] = gnss_burst
    lat = []
    orig = getattr(mod, 'Pipeline', None)
    if orig is not None:
        mod.Pipeline = _timed_pipeline(orig, lat)
    try:
        c0, w0 = time.process_time(), time.perf_counter()
        est = fn(ev.cache_path(bag), params, **kw)
        cpu, wall = time.process_time() - c0, time.perf_counter() - w0
    finally:
        if orig is not None:
            mod.Pipeline = orig
    return est, cpu, wall, (np.array(lat) if orig is not None else None)


# ------------------------------------------------------------------ one bag
def output_checks(est, D):
    t = np.asarray(est['t'], float)
    v = np.asarray(est['v'], float)
    hp = np.asarray(est.get('has_pose', np.ones(len(t), bool)), bool)
    xyz = np.column_stack([np.asarray(est[k], float) for k in ('x', 'y', 'z')]) if len(t) else np.zeros((0, 3))
    stamps = np.concatenate([np.zeros(0)] + [np.asarray((D.get(tp) or {}).get('t_hdr', []), float) for tp in INPUT_TOPICS])
    # robust input span (a few glitched header stamps must not inflate it)
    span = float(np.percentile(stamps, 99.9) - np.percentile(stamps, 0.1)) if len(stamps) > 1 else 0.0
    c = dict(n_out=int(len(t)), n_in=int(len(stamps)),
             n_nonfinite_v=int((~np.isfinite(v)).sum()), n_neg_v=int((v < 0).sum()),
             v_max=float(np.nanmax(v)) if len(v) else None,
             pose_frac=float(hp.mean()) if len(hp) else 0.0,
             n_nonfinite_pose=int((hp & ~np.isfinite(xyz).all(axis=1)).sum()),
             stamps_from_input=float(np.isin(t, stamps).mean()) if len(t) else None,
             pub_rate_hz=float(len(t) / span) if span > 0 else None)
    if hp.sum() > 1:
        tp, P = t[hp], xyz[hp]
        o = np.argsort(tp, kind='stable')
        step = np.hypot(*np.diff(P[o, :2], axis=0).T)
        c['max_step_m'] = float(np.nanmax(step)) if len(step) else 0.0
        c['first_pose_s'] = float(tp.min() - t.min())
    return c


def process_bag(bag, info, cfg):
    rec = dict(bag=bag, vehicle=info.get('vehicle'), group=info.get('group'), fold=info.get('fold'),
               eval_ok=bool(info.get('eval_ok')), dup_of=info.get('dup_of'), has_gnss=info.get('has_gnss'),
               direction_ref=info.get('direction'), status='ok', error=None)
    try:
        D = ev.load_cache(bag)
        params, extra = build_params(bag, info.get('fold', 0), cfg)
        rec['extra_params'] = extra
        est, cpu, wall, lat = run_estimator(cfg['estimator'], bag, params, D, cfg['gnss_limit'], cfg.get('gnss_burst'))
        rec.update(cpu_s=cpu, wall_s=wall)
        rec.update(output_checks(est, D))
        rec['cpu_us_per_msg'] = 1e6 * float(est.get('cpu_s', cpu)) / max(rec['n_in'], 1)
        if lat is not None and len(lat):
            rec.update(lat_ms_p50=1e3 * float(np.median(lat)), lat_ms_p99=1e3 * float(np.percentile(lat, 99)),
                       lat_ms_max=1e3 * float(lat.max()))
        for k in ('direction', 'init_kind'):
            if k in est:
                val = est[k]
                rec['est_' + k] = val.item() if hasattr(val, 'item') and np.ndim(val) == 0 else (
                    None if val is None else str(val))
        rec['metrics'] = {}
        series = None
        for ant in cfg['antennas']:
            ref = ev.load_ref(bag, ant, cfg['clean_ref'], D=D)
            if ref is None:
                continue
            want_series = cfg['plot_all'] or bag in cfg['plot_bags']
            R = ev.evaluate(bag, est, ant, cfg['tol'], cfg['gate'], ref=ref, series=want_series and series is None)
            series = R.pop('series', series)
            rec['metrics'][ant] = R
        if cfg.get('save_est'):
            d = os.path.join(cfg['out'], 'est')
            os.makedirs(d, exist_ok=True)
            np.savez_compressed(os.path.join(d, bag + '.npz'),
                                **{k: v for k, v in est.items() if isinstance(v, np.ndarray) and v.dtype != object})
        if series is not None:
            import plots
            plots.plot_bag(bag, series, os.path.join(cfg['out'], 'plots', bag + '.png'),
                           metrics=rec['metrics'].get(cfg['antennas'][0]), title_extra=cfg['tag'])
    except Exception:
        rec['status'] = 'error'
        rec['error'] = traceback.format_exc(limit=8)
    return rec


# ------------------------------------------------------------------ bag selection
def select_bags(spec, index):
    """Comma-separated union of selectors; a selector may intersect terms with '&' (eval&vehicle:30639).

    Terms: all | eval | unique | nogps (no GNSS fix topic) | noeval (not eval_ok) | vehicle:<v> | group:<g> | fold:<k> | dir:T2S|S2T |
    @file (one bag per line) | bag id (the hash suffix is enough)."""
    B = index['bags']

    def term(tok):
        if tok == 'all':
            return set(B)
        if tok == 'eval':
            return {b for b, r in B.items() if r['eval_ok']}
        if tok == 'unique':
            return {b for b, r in B.items() if r['dup_of'] is None}
        if tok == 'nogps':
            return {b for b, r in B.items() if not r['has_gnss']}
        if tok == 'noeval':
            return {b for b, r in B.items() if not r['eval_ok']}
        if tok.startswith('@'):
            with open(tok[1:]) as f:
                return {l.strip() for l in f if l.strip() and not l.startswith('#')}
        if ':' in tok:
            key, val = tok.split(':', 1)
            field = {'vehicle': 'vehicle', 'group': 'group', 'fold': 'fold', 'dir': 'direction'}[key]
            return {b for b, r in B.items() if str(r[field]) == val}
        m = {b for b in B if b == tok or b.endswith('_' + tok)}
        if not m:
            raise SystemExit('unknown bag %r' % tok)
        return m

    out = []
    for sel in spec.split(','):
        sel = sel.strip()
        if not sel:
            continue
        parts = [term(t.strip()) for t in sel.split('&')]
        out += sorted(set.intersection(*parts))
    seen = set()
    return [b for b in out if not (b in seen or seen.add(b))]


# ------------------------------------------------------------------ summaries
def _fmt(x, nd=3):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return '-'
    if isinstance(x, float):
        return ('%.' + str(nd) + 'g') % x if abs(x) < 1e4 else '%.0f' % x
    return str(x)


def summarize(recs, cfg):
    ant = cfg['antennas'][0]
    scored = [r for r in recs if r['status'] == 'ok' and r['eval_ok'] and ant in r.get('metrics', {})]
    M = [r['metrics'][ant] for r in scored]
    S = dict(tag=cfg['tag'], antenna=ant, n_bags=len(recs), n_scored=len(M),
             n_errors=sum(r['status'] != 'ok' for r in recs),
             errors={r['bag']: r['error'].strip().splitlines()[-1] for r in recs if r['status'] != 'ok'},
             extra_params=sorted({k for r in recs for k in r.get('extra_params', [])}),
             overall=ev.aggregate(M))
    for key, field in (('by_direction', 'dir'), ('by_vehicle', 'vehicle'), ('by_group', 'group')):
        vals = sorted({(m.get(field) if field == 'dir' else r[field]) for r, m in zip(scored, M)}, key=str)
        S[key] = {str(v): ev.aggregate([m for r, m in zip(scored, M) if (m.get(field) if field == 'dir' else r[field]) == v])
                  for v in vals}
    for ant2 in cfg['antennas'][1:]:
        M2 = [r['metrics'][ant2] for r in scored if ant2 in r['metrics']]
        S['antenna_' + ant2] = ev.aggregate(M2)
    # largest |value| first; NaN (metric undefined for the bag) last
    S['worst'] = {k: [(r['bag'], m.get(k)) for r, m in sorted(zip(scored, M), key=lambda rm: -np.nan_to_num(abs(rm[1].get(k, np.nan)), nan=-1.0))[:8]]
                  for k in WORST_BY}
    ok = [r for r in recs if r['status'] == 'ok']
    rob = {}
    for k in ('cpu_us_per_msg', 'lat_ms_p99', 'lat_ms_max', 'pub_rate_hz', 'stamps_from_input', 'pose_frac', 'max_step_m'):
        vals = np.array([r.get(k) for r in ok if r.get(k) is not None], float)
        if len(vals):
            rob[k] = dict(median=float(np.median(vals)), min=float(vals.min()), max=float(vals.max()))
    rob['n_nonfinite_v'] = int(sum(r.get('n_nonfinite_v', 0) for r in ok))
    rob['n_nonfinite_pose'] = int(sum(r.get('n_nonfinite_pose', 0) for r in ok))
    rob['n_neg_v'] = int(sum(r.get('n_neg_v', 0) for r in ok))
    rob['bags_without_pose'] = sorted(r['bag'] for r in ok if r.get('pose_frac', 0) == 0)
    rob['total_cpu_s'] = float(sum(r.get('cpu_s', 0) for r in ok))
    rob['total_msgs'] = int(sum(r.get('n_in', 0) for r in ok))
    S['robustness'] = rob
    return S


def summary_md(S, cfg):
    L = ['# %s' % S['tag'], '',
         'Estimator `%s`, reference antenna **%s**, GNSS limit %s s%s, tol %.3f s, gate %.1f m%s.' % (
             cfg['estimator'], S['antenna'], cfg['gnss_limit'],
             ' + %.1f s bursts every %.0f s' % (cfg['gnss_burst'][1], cfg['gnss_burst'][0]) if cfg.get('gnss_burst') else '',
             cfg['tol'], cfg['gate'],
             ', cleaned reference' if cfg['clean_ref'] else ''),
         'Params: %s; overrides: %s%s.' % (cfg.get('params') or 'defaults', cfg.get('overrides') or '{}',
                                          '; fold params ' + cfg['fold_params'] if cfg.get('fold_params') else ''),
         '', 'Bags run: %d, errors: %d, scored (eval_ok): %d.' % (S['n_bags'], S['n_errors'], S['n_scored'])]
    if S['extra_params']:
        L.append('Parameters not in `Params` (set as attributes): %s.' % ', '.join(S['extra_params']))
    if S['errors']:
        L += ['', '## Exceptions', ''] + ['- `%s`: %s' % kv for kv in S['errors'].items()]
    L += ['', '## Overall (|value| over eval_ok bags; signed median in brackets)', '',
          '| metric | median | mean | p90 | max |', '|---|---|---|---|---|']
    for k in TABLE_KEYS:
        a = S['overall'].get(k)
        if not a or not a['n']:
            continue
        med = _fmt(a['median']) + (' (%s)' % _fmt(a['median_signed']) if 'median_signed' in a else '')
        L.append('| %s | %s | %s | %s | %s |' % (k, med, _fmt(a['mean']), _fmt(a['p90']), _fmt(a['max'])))
    for title, key in (('By direction', 'by_direction'), ('By vehicle', 'by_vehicle'), ('By group (fold)', 'by_group')):
        L += ['', '## %s (medians)' % title, '', '| %s | n | %s |' % (key[3:], ' | '.join(GROUP_KEYS)),
              '|---|---|' + '---|' * len(GROUP_KEYS)]
        for v, agg in S[key].items():
            L.append('| %s | %d | %s |' % (v, agg['score_loss']['n'], ' | '.join(_fmt(agg[k]['median']) for k in GROUP_KEYS)))
    for k in S:
        if k.startswith('antenna_'):
            agg = S[k]
            L += ['', '## Against %s reference (medians)' % k[8:], '',
                  ' '.join('%s %s;' % (m, _fmt(agg[m]['median'])) for m in ('e3d_mean', 'al_mean', 'al_mean_abs', 'al_rmse'))]
    L += ['', '## Worst bags', '']
    for k, rows in S['worst'].items():
        L.append('- **%s**: %s' % (k, ', '.join('%s %s' % (b, _fmt(v)) for b, v in rows)))
    R = S['robustness']
    L += ['', '## Robustness and timing (all bags that ran)', '']
    for k, v in R.items():
        if isinstance(v, dict):
            L.append('- %s: median %s, min %s, max %s' % (k, _fmt(v['median']), _fmt(v['min']), _fmt(v['max'])))
        elif isinstance(v, list):
            L.append('- %s (%d): %s' % (k, len(v), ', '.join(v) if v else '-'))
        else:
            L.append('- %s: %s' % (k, _fmt(v) if isinstance(v, float) else v))
    return '\n'.join(L) + '\n'


def compare(tag_a, tag_b, antenna='master', keys=GROUP_KEYS + ('v_cov', 'p_cov', 'e2d_offmap_mean')):
    A = {r['bag']: r for r in json.load(open(os.path.join(RESULTS, tag_a, 'per_bag.json')))}
    B = {r['bag']: r for r in json.load(open(os.path.join(RESULTS, tag_b, 'per_bag.json')))}
    common = sorted(b for b in A if b in B and A[b]['eval_ok'] and antenna in (A[b].get('metrics') or {})
                    and antenna in (B[b].get('metrics') or {}))
    print('%d common eval_ok bags; %s -> %s' % (len(common), tag_a, tag_b))
    print('%-16s %10s %10s %10s %6s' % ('metric', 'med A', 'med B', 'med B-A', 'B<A'))
    for k in keys:
        a = np.array([abs(A[b]['metrics'][antenna].get(k) if A[b]['metrics'][antenna].get(k) is not None else np.nan) for b in common])
        b_ = np.array([abs(B[b]['metrics'][antenna].get(k) if B[b]['metrics'][antenna].get(k) is not None else np.nan) for b in common])
        m = np.isfinite(a) & np.isfinite(b_)
        if not m.any():
            continue
        print('%-16s %10.4g %10.4g %10.4g %3d/%-3d' % (k, np.median(a[m]), np.median(b_[m]), np.median(b_[m] - a[m]),
                                                      int((b_[m] < a[m]).sum()), int(m.sum())))


def _jsonable(x):
    return ev._jsonable(x)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', help='results/<tag>/')
    ap.add_argument('--bags', default='all', help='all | eval | unique | nogps | noeval | vehicle:30639 | group:<g> | fold:<k> | '
                                                  'dir:T2S | @file | bag ids (hash suffix is enough); comma = union, '
                                                  '& = intersection (eval&dir:S2T)')
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--params', help='params yaml (plain mapping or ROS 2 ros__parameters layout)')
    ap.add_argument('--fold-params', help='directory with fold_<k>.yaml applied per bag fold (out-of-fold calibration)')
    ap.add_argument('--set', nargs='*', default=[], help='key=value parameter overrides')
    ap.add_argument('--estimator', default=DEFAULT_ESTIMATOR,
                    help='module:function or file.py:function, called as f(bag_path, params, data=..[, gnss_limit_s=..])')
    ap.add_argument('--antenna', default='master', help='master | rover | mid | all (first one is primary)')
    ap.add_argument('--gnss-limit', type=float, default=10.0, help='feed GNSS only this long after the first fix (s)')
    ap.add_argument('--gnss-burst', type=float, nargs=2, metavar=('PERIOD', 'DUR'), default=None,
                    help='after --gnss-limit also feed a DUR-second GNSS burst every PERIOD seconds (GNSS aiding)')
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--gate', type=float, default=5.0)
    ap.add_argument('--clean-ref', action='store_true', help='drop reference glitches (see evaluate.py)')
    ap.add_argument('--plots', default='none', help='none | all | worst | comma list of bags')
    ap.add_argument('--save-est', action='store_true', help='save results/<tag>/est/<bag>.npz')
    ap.add_argument('--index', default=bi.INDEX_PATH)
    ap.add_argument('--compare', nargs=2, metavar=('TAG_A', 'TAG_B'))
    a = ap.parse_args(argv)
    if a.compare:
        compare(*a.compare, antenna=a.antenna if a.antenna != 'all' else 'master')
        return 0
    if not a.tag:
        ap.error('--tag is required')
    if not os.path.exists(a.index):
        idx = bi.build_index(bi.list_cached_bags(), a.jobs)
        with open(a.index, 'w') as f:
            json.dump(idx, f, indent=1)
    index = bi.load_index(a.index)
    bags = select_bags(a.bags, index)
    out = os.path.join(RESULTS, a.tag)
    os.makedirs(out, exist_ok=True)
    antennas = ['master', 'rover', 'mid'] if a.antenna == 'all' else a.antenna.split(',')
    plot_bags = set() if a.plots in ('none', 'worst', 'all') else set(select_bags(a.plots, index))
    cfg = dict(tag=a.tag, out=out, params=a.params, fold_params=a.fold_params, overrides=parse_sets(a.set),
               estimator=a.estimator, antennas=antennas, gnss_limit=a.gnss_limit, gnss_burst=a.gnss_burst, tol=a.tol, gate=a.gate,
               clean_ref=a.clean_ref, plot_all=a.plots == 'all', plot_bags=plot_bags, save_est=a.save_est)
    t0 = time.time()
    order = sorted(bags, key=lambda b: -index['bags'].get(b, {}).get('duration_s', 0))
    recs = []
    with ProcessPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(process_bag, b, index['bags'].get(b, {}), cfg): b for b in order}
        for i, f in enumerate(as_completed(futs), 1):
            r = f.result()
            recs.append(r)
            m = (r.get('metrics') or {}).get(antennas[0], {})
            print('[%3d/%d] %-16s %-5s v_rmse=%-7s al_rmse=%-7s e3d=%-7s cpu=%sus/msg%s' % (
                i, len(bags), r['bag'], r['status'], _fmt(m.get('v_rmse')), _fmt(m.get('al_rmse')),
                _fmt(m.get('e3d_mean')), _fmt(r.get('cpu_us_per_msg')), '' if r['status'] == 'ok' else '  <- ' +
                r['error'].strip().splitlines()[-1]), flush=True)
    recs.sort(key=lambda r: r['bag'])
    S = summarize(recs, cfg)
    if a.plots == 'worst':
        worst = []
        for k in ('score_loss', 'al_rmse', 'v_rmse'):
            worst += [b for b, _ in S['worst'][k][:3] if b not in worst]
        cfg['plot_bags'] = set(worst)
        with ProcessPoolExecutor(a.jobs) as ex:
            list(ex.map(process_bag, worst, [index['bags'][b] for b in worst], [cfg] * len(worst)))
    S['wall_s'] = time.time() - t0
    cfg_out = {k: (sorted(v) if isinstance(v, set) else v) for k, v in cfg.items()}
    cfg_out['out'] = os.path.relpath(out, ev.ROOT)   # no machine-specific absolute paths in results
    with open(os.path.join(out, 'per_bag.json'), 'w') as f:
        json.dump(_jsonable(recs), f, indent=1)
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(_jsonable(dict(S, config=cfg_out, argv=sys.argv)), f, indent=1)
    md = summary_md(S, cfg)
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write(md)
    print(md)
    print('-> %s (%.0f s)' % (out, S['wall_s']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
