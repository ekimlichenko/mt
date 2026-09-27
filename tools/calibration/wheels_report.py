"""Summary of the pickle written by wheels_validate.py (per robustness bag, per variant, per group).

    python tools/calibration/wheels_report.py [path/to/validate.pkl]
"""
import os
import pickle
import sys

import numpy as np

import _common as C
from wheels_validate import DROPOUT, SLIP, STAMP


def cell(r, k, suf=''):
    x = r.get(k, {})
    return '%.4f/%.3f' % (x['rmse' + suf], x['max' + suf]) if ('rmse' + suf) in x else '   -   '


def med(x):
    return float(np.median(x)) if len(x) else float('nan')


def main():
    fn = sys.argv[1] if len(sys.argv) > 1 else os.path.join(C.WORK, 'wheels', 'validate.pkl')
    res = pickle.load(open(fn, 'rb'))
    by = {r['bag']: r for r in res}
    V = [k for k in res[0] if k.startswith('W') and isinstance(res[0][k], dict)] + (['B6'] if 'B6' in res[0] else [])
    refb = [r for r in res if r['has_ref'] and r['W'].get('n', 0) > 100]
    refn = {r['bag'] for r in refb}
    show = [k for k in ('W', 'W_pre', 'W_naive', 'B6') if k in V]
    for title, L in (('DROPOUT', DROPOUT), ('SLIP', SLIP), ('TIMESTAMP', STAMP)):
        L = [b for b in L if b in by]
        if not L:
            continue
        print('-- %s   rmse/max [m/s] (%s)' % (title, ', '.join(show)))
        for b in L:
            r = by[b]
            if b not in refn:
                print('  %-16s no status-2 RTK reference; diag %s' % (b, {k: r['W']['diag'][k] for k in ('tg_counts', 'n_disagree', 'n_stuck_zero')}))
                continue
            line = '  %-16s n=%6d  ' % (b, r['W']['n']) + '  '.join('%-7s %s' % (k, cell(r, k)) for k in show)
            if title == 'TIMESTAMP':
                line += ' | published only (no late): W %s B6 %s' % (cell(r, 'W', '_valid'), cell(r, 'B6', '_valid'))
            if abs(r['W'].get('rmse_clean', 0) - r['W']['rmse']) > 1e-3:
                line += ' | GNSS-clean: W %s B6 %s' % (cell(r, 'W', '_clean'), cell(r, 'B6', '_clean'))
            print(line)

    print('\n-- all %d bags with a status-2 RTK reference (of %d)' % (len(refb), len(res)))
    print('%-11s %8s %8s %8s %9s %9s | median rmse: dropout / slip / other' % ('variant', 'med rmse', 'mean', 'pooled', 'med max', 'worst max'))
    oth = [r for r in refb if r['bag'] not in DROPOUT + SLIP + STAMP]
    for k in V:
        rr = [r for r in refb if 'rmse' in r.get(k, {})]
        if not rr:
            continue
        rm = np.array([r[k]['rmse'] for r in rr]); mx = np.array([r[k]['max'] for r in rr])
        n = np.array([r[k]['n'] for r in rr])
        g = [med([by[b][k]['rmse'] for b in L if b in refn]) for L in (DROPOUT, SLIP)] + [med([r[k]['rmse'] for r in oth])]
        print('%-11s %8.4f %8.4f %8.4f %9.3f %9.3f | %.4f / %.4f / %.4f' % (
            k, np.median(rm), rm.mean(), np.sqrt((rm ** 2 * n).sum() / n.sum()), np.median(mx), mx.max(), *g))

    if 'B6' in V and refb:
        d = np.array([r['W']['rmse'] - r['B6']['rmse'] for r in refb])
        print('\nW vs B6 (non-causal, header-sorted) per bag: better %d, |d|<1e-4 %d, worse %d (median d %+.4f, max %+.4f)' % (
            (d < -1e-4).sum(), (abs(d) <= 1e-4).sum(), (d > 1e-4).sum(), np.median(d), d.max()))
        for k in ('W', 'B6'):
            a = [r[k]['rmse_noslip'] for r in refb if 'rmse_noslip' in r[k]]
            b = [r[k]['rmse_slip'] for r in refb if 'rmse_slip' in r[k]]
            print('%-3s median rmse on samples W does not flag slip %.4f (%d bags); on W-slip samples %.4f (%d bags)' % (
                k, med(a), len(a), med(b), len(b)))
    if 'W_naive' in V and refb:
        dn = np.array([r['W']['rmse'] - r['W_naive']['rmse'] for r in refb])
        print('W vs W_naive (causal B6)      per bag: better %d, |d|<1e-4 %d, worse %d (median d %+.4f, max %+.4f)' % (
            (dn < -1e-4).sum(), (abs(dn) <= 1e-4).sum(), (dn > 1e-4).sum(), np.median(dn), dn.max()))
    ns = sum(r['W']['n_slip'] for r in res); ne = sum(r['W']['n_events'] for r in res)
    print('W slip-flagged wheel events: %d of %d (%.2f%%)' % (ns, ne, 100.0 * ns / max(ne, 1)))
    keys = ['spikes', 'rejected', 'n_disagree', 'n_uncertain', 'n_accel_flag', 'n_lock', 'n_frozen', 'n_stuck_zero']
    for k in V:
        if k == 'B6':
            continue
        print('%-10s diag totals: %s  late %d' % (k, {q: sum(r[k]['diag'][q] for r in res) for q in keys},
                                                 sum(r[k]['n_late'] for r in res)))


if __name__ == '__main__':
    main()
