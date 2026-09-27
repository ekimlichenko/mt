#!/usr/bin/env python3
"""Per-bag diagnostic figure: results/<tag>/plots/<bag>.png.

Left column (shared time axis, seconds since the first reference sample):
  1. speed: GNSS reference vs estimate;
  2. speed error of the matched pairs (estimate - reference);
  3. along-track error on the map (estimate ahead > 0) and 2D error everywhere.
Right: xy map (pathgraph frame) with both routes (thick = T2S), the RTK
reference track and the estimated track; below it zooms on the first and the
last 600 m of the reference path (the off-map terminus sections).

``plot_bag`` takes the ``series`` dict of ``evaluate.evaluate(..., series=True)``.
From the command line the estimator is re-run through ``run_cv.process_bag``:

    python tools/plots.py --tag mytag --bags 30618_0e41eac3,40ffd323 [--set key=value ...]
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as ev  # noqa: E402

C_REF, C_EST, C_ERR, C_MAP = '#1f77b4', '#d62728', '#555555', '#c8c8c8'
ZOOM_M = 600.0


def _draw_xy(ax, S):
    for name, rt in ev.routes().items():
        ax.plot(rt.P[:, 0], rt.P[:, 1], color=C_MAP, lw=3 if name == 'T2S' else 1.5, zorder=1)
    if len(S.get('ref_tp_all', [])):
        P = S['ref_xyz_all']
        d = _decimate(len(P))
        ax.plot(P[d, 0], P[d, 1], '.', color=C_REF, ms=1.5, label='reference (RTK)', zorder=2)
        ax.plot(P[0, 0], P[0, 1], 'o', color=C_REF, ms=6, zorder=4)
    if len(S.get('est_t_pose', [])):
        P = S['est_xyz_pose']
        d = _decimate(len(P))
        ax.plot(P[d, 0], P[d, 1], '-', color=C_EST, lw=0.9, label='estimate', zorder=3)
        ax.plot(P[0, 0], P[0, 1], 's', color=C_EST, ms=5, zorder=4)
    ax.set_aspect('equal', adjustable='box')
    ax.tick_params(labelsize=7)


def _decimate(n, max_pts=6000):
    return slice(None, None, max(1, n // max_pts))


def plot_bag(bag, S, out_png, metrics=None, title_extra=''):
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig = plt.figure(figsize=(16, 9.5))
    gs = fig.add_gridspec(3, 3, width_ratios=(1.6, 0.8, 0.8), hspace=0.3, wspace=0.22)
    ax_v = fig.add_subplot(gs[0, 0])
    ax_ve = fig.add_subplot(gs[1, 0], sharex=ax_v)
    ax_al = fig.add_subplot(gs[2, 0], sharex=ax_v)
    ax_xy = fig.add_subplot(gs[0:2, 1:3])
    t0 = S['tv'][0] if len(S.get('tv', [])) else (S['tp'][0] if len(S.get('tp', [])) else 0.0)

    if len(S.get('tv', [])):
        d = _decimate(len(S['tv']))
        ax_v.plot(S['tv'][d] - t0, S['v_ref'][d], color=C_REF, lw=1.0, label='GNSS reference')
    if len(S.get('v_est_t', [])):
        d = _decimate(len(S['v_est_t']))
        ax_v.plot(S['v_est_t'][d] - t0, S['v_est'][d], color=C_EST, lw=0.8, alpha=0.8, label='estimate')
    ax_v.set_ylabel('speed, m/s')
    ax_v.legend(loc='upper right', fontsize=8)

    if len(S.get('tv', [])):
        d = _decimate(len(S['tv']))
        ax_ve.plot(S['tv'][d] - t0, S['v_err'][d], color=C_ERR, lw=0.7)
        ax_ve.axhline(0, color='k', lw=0.5)
        lim = np.nanpercentile(np.abs(S['v_err']), 99.5) if np.isfinite(S['v_err']).any() else 1.0
        ax_ve.set_ylim(-max(lim * 1.3, 0.1), max(lim * 1.3, 0.1))
    ax_ve.set_ylabel('speed error, m/s')

    if len(S.get('tp', [])):
        d = _decimate(len(S['tp']))
        ax_al.plot(S['tp'][d] - t0, S['e2d'][d], color=C_MAP, lw=1.0, label='2D error (all)')
        ax_al.plot(S['tp'][d] - t0, S['al'][d], color=C_EST, lw=1.0, label='along-track (on map)')
        ax_al.plot(S['tp'][d] - t0, S['ct'][d], color=C_REF, lw=0.8, label='cross-track (on map)')
        ax_al.axhline(0, color='k', lw=0.5)
        ax_al.legend(loc='best', fontsize=8)
    ax_al.set_ylabel('position error, m')
    ax_al.set_xlabel('time since first reference sample, s')

    ax_start = fig.add_subplot(gs[2, 1])
    ax_end = fig.add_subplot(gs[2, 2])
    for ax in (ax_xy, ax_start, ax_end):
        _draw_xy(ax, S)
    Pr = S.get('ref_xyz_all', np.zeros((0, 3)))
    if len(Pr):
        pad = 100.0
        ax_xy.set_xlim(np.nanmin(Pr[:, 0]) - pad, np.nanmax(Pr[:, 0]) + pad)
        ax_xy.set_ylim(np.nanmin(Pr[:, 1]) - pad, np.nanmax(Pr[:, 1]) + pad)
        cum = np.r_[0.0, np.cumsum(np.hypot(*np.diff(Pr[:, :2], axis=0).T))]
        for ax, sel, name in ((ax_start, cum <= ZOOM_M, 'start'), (ax_end, cum >= cum[-1] - ZOOM_M, 'end')):
            Q = Pr[sel]
            cx, cy = (Q[:, 0].min() + Q[:, 0].max()) / 2, (Q[:, 1].min() + Q[:, 1].max()) / 2
            half = max(np.ptp(Q[:, 0]), np.ptp(Q[:, 1])) / 2 + 30.0
            ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
            ax.set_title('%s (first/last %.0f m of reference)' % (name, ZOOM_M) if name == 'start' else name, fontsize=8)
    ax_xy.legend(loc='best', fontsize=8)
    ax_xy.set_title('map frame (UTM37N - (300000, 6100000)); o = start of reference, s = first pose', fontsize=8)
    for ax in (ax_v, ax_ve, ax_al, ax_xy, ax_start, ax_end):
        ax.grid(alpha=0.3)

    title = bag + ('  [%s]' % title_extra if title_extra else '')
    if metrics:
        g = lambda k: metrics.get(k, np.nan)
        title += ('\n%s  v_rmse %.3f  bias %.3f  |  e3d mean %.2f  al mean|.| %.2f rmse %.2f max %.1f  ct rmse %.2f  '
                  'drift %.2f%%  off-map e2d %.1f  |  loss %.2f') % (
            g('dir'), g('v_rmse'), g('v_bias'), g('e3d_mean'), g('al_mean_abs'), g('al_rmse'), g('al_max'),
            g('ct_rmse'), g('drift_pct'), g('e2d_offmap_mean'), g('score_loss'))
    fig.suptitle(title, fontsize=10)
    fig.savefig(out_png, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return out_png


def main(argv=None):
    import run_cv
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--bags', required=True, help='same selectors as run_cv.py --bags')
    ap.add_argument('--params')
    ap.add_argument('--set', nargs='*', default=[])
    ap.add_argument('--estimator', default=run_cv.DEFAULT_ESTIMATOR)
    ap.add_argument('--antenna', default='master')
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    a = ap.parse_args(argv)
    index = run_cv.bi.load_index()
    bags = run_cv.select_bags(a.bags, index)
    out = os.path.join(run_cv.RESULTS, a.tag)
    cfg = dict(tag=a.tag, out=out, params=a.params, fold_params=None, overrides=run_cv.parse_sets(a.set),
               estimator=a.estimator, antennas=[a.antenna], gnss_limit=a.gnss_limit, tol=0.05, gate=5.0,
               clean_ref=False, plot_all=True, plot_bags=set(), save_est=False)
    for b in bags:
        r = run_cv.process_bag(b, index['bags'][b], cfg)
        print(b, r['status'], os.path.join(out, 'plots', b + '.png') if r['status'] == 'ok' else r['error'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
