"""TimeGuard thresholds (time_glitch_s, time_jump_margin_s, time_late_tol_s, time_consensus_n,
time_resync_n): header-stamp statistics and the flags TimeGuard raises on the recorded bags.

Per bag (vehicle streams front / rear / cmd, record order):
  jitter   |(t_rec - t_hdr) - running median over 51 messages| of every stream: normal transport
           jitter vs the +-0.4..1.0 s header glitches of the STAMP bags   -> time_glitch_s (0.3 s)
  lateness max header stamp seen so far (all streams) - t_hdr of the arriving message: how
           far a message is behind the others in record order          -> time_late_tol_s (0.15 s)
  flags    TimeGuard(Params + --set) counts [ok, forward, late, invalid] in the first BURST s of
           record time (startup reordering) and after it; resyncs.  Outside the STAMP bags almost
           nothing may be flagged after the burst.
time_late_tol_s is also scored on the speed error by wheels_validate.py (variants W_late005,
W_late030); consensus_n / resync_n are engineering choices (3 s / 2 s of 10 Hz messages).

    python tools/calibration/timeguard_flags.py [--bags N|b1,b2] [--set time_glitch_s=0.5 ...] [--jobs 4]
"""
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.ndimage import median_filter

import _common as C
from tram_backup_odometry.core.preprocess import TimeGuard
from wheels_validate import STAMP, events

BURST = 1.0


def one(args):
    b, sets = args
    D = C.load_cache(b)
    ev = events(D)
    out = {'bag': b}
    jit = []
    for top in (C.FT, C.RT, C.CT):
        x = D.get(top) or {}
        if len(x.get('t_hdr', [])) < 60:
            continue
        o = np.argsort(x['t_rec'], kind='stable')
        off = (np.asarray(x['t_rec'], float) - np.asarray(x['t_hdr'], float))[o]
        jit.append(np.abs(off - median_filter(off, size=51, mode='nearest')))
    out['jitter'] = np.concatenate(jit) if jit else np.zeros(0)
    lat = np.empty(len(ev))
    m = -np.inf
    t0 = ev[0][0] if ev else 0.0
    for i, (tr, th, s, _) in enumerate(ev):
        lat[i] = max(m - th, 0.0) if tr - t0 > BURST else np.nan
        m = max(m, th)
    out['late'] = lat[np.isfinite(lat)]
    tg = TimeGuard(C.params_for(b, sets))
    t_start = ev[0][0] if ev else 0.0
    c = np.zeros((2, 4), int)
    per_stream, t_flag = {}, []
    for tr, th, s, _ in ev:
        _, f = tg.correct(s, th)
        after = tr - t_start > BURST
        c[int(after), int(f)] += 1
        if after and f:
            per_stream[(s, f)] = per_stream.get((s, f), 0) + 1
            t_flag.append(tr - t_start)
    out.update(counts=c, per_stream=per_stream, resyncs=tg.resyncs, t_flag=t_flag)
    return out


def main():
    t_wall = time.time()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='', help="'' = all unique bags, N = first N, or a comma list")
    ap.add_argument('--set', nargs='*', default=[])
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    bags = C.select_bags(a.bags, C.unique_bags())
    jobs = [(b, a.set) for b in bags]
    if a.jobs > 1:
        with ProcessPoolExecutor(a.jobs) as ex:
            res = list(ex.map(one, jobs))
    else:
        res = [one(j) for j in jobs]
    normal = [r for r in res if r['bag'] not in STAMP]
    glitch = [r for r in res if r['bag'] in STAMP]
    jn = np.concatenate([r['jitter'] for r in normal]) if normal else np.zeros(0)
    print('-- header/record offset jitter |off - running median| [s]')
    if len(jn):
        print('  normal bags (%d): p99 %.3f p99.9 %.3f p99.99 %.3f max %.3f' % (
            len(normal), *np.percentile(jn, (99, 99.9, 99.99)), jn.max()))
    for r in glitch:
        j = r['jitter']
        print('  glitch bag %-16s max %.2f s, messages > 0.3 s: %d' % (r['bag'], j.max() if len(j) else 0, (j > 0.3).sum()))
    ln = np.concatenate([r['late'] for r in normal]) if normal else np.zeros(0)
    if len(ln):
        print('-- lateness in record order after the first %.1f s (max header so far - header) [s], normal bags:' % BURST)
        print('  p99 %.3f p99.9 %.3f p99.99 %.3f max %.3f; messages > 0.05 s: %.3f%%, > 0.15 s: %.4f%%' % (
            *np.percentile(ln, (99, 99.9, 99.99)), ln.max(), 100 * np.mean(ln > 0.05), 100 * np.mean(ln > 0.15)))
    print('-- TimeGuard flags [ok, forward, late, invalid] (Params %s), after the first %.1f s of record time:' % (a.set or 'defaults', BURST))
    tot = np.zeros((2, 4), int)
    for r in res:
        c = r['counts']
        if r['bag'] not in STAMP:
            tot += c
        if c[1, 1:].sum() or r['bag'] in STAMP:
            tl = r['t_flag']
            span = '%.1f..%.1f s' % (min(tl), max(tl)) if tl else '-'
            print('  %s%-16s burst %s  after %s  resyncs %d  %s  at %s' % (
                '*' if r['bag'] in STAMP else ' ', r['bag'], c[0].tolist(), c[1].tolist(), r['resyncs'], r['per_stream'], span))
    print('  normal bags (%d): burst %s   after burst %s   (%.4f%% flagged after burst)' % (
        len(normal), tot[0].tolist(), tot[1].tolist(), 100.0 * tot[1, 1:].sum() / max(tot[1].sum(), 1)))
    out = C.out_dir('timeguard')
    C.save_json(os.path.join(out, 'summary.json'), dict(
        sets=a.set, bags=bags, jitter_p9999=float(np.percentile(jn, 99.99)) if len(jn) else None,
        jitter_max=float(jn.max()) if len(jn) else None,
        late_p9999=float(np.percentile(ln, 99.99)) if len(ln) else None,
        flags_after_burst=tot[1].tolist(), flags_burst=tot[0].tolist()))
    print('written %s (%.0f s)' % (os.path.join(out, 'summary.json'), time.time() - t_wall))


if __name__ == '__main__':
    main()
