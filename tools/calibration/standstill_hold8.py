"""Standstill latch and the notch -8 hold mode (analysis_notes D6, D8; Params start_delay_s,
standstill_v, hold8_notch, hold8_v, hold8_low_acc, sigma_a_hold8).

Reconstruction (the original statistics were computed inline).  Unique bags with wheels + cmd;
v = mean of the de-duplicated front / rear wheel speed / K(vehicle) on a 20 Hz header-time grid,
a = its 1 s central difference, n = notch (zero-order hold on the cmd header stamps).

  starts  cmd onset of the first traction notch (n > 0 after n <= 0) while the wheels are at rest
          (v < 0.05 over the previous 2 s); delay = first v > 0.1 m/s after the onset (within 10 s)
          -> start_delay_s (1.2 s ~ median; the v > 0.1 crossing lags the first motion by 0.1-0.2 s
          of the jerk-limited ramp, and fresh wheel motion overrides the latch in the filter);
          onsets of motion without a traction notch in the previous 5 s (the -8 mode starts)
          -> hold8_rest_grow / latch_needs_wheels
  stops   v < 0.05 for >= 2 s: notch during the stop, last notch before it, dv over the last 1 s
  -8      contiguous n == -8 intervals: long ones (> 5 s) with entry / exit notch, exit speed,
          duration; mean a in speed bins over -8 samples older than 1 s
          -> hold8_v (2.5: a ~ 0 above, ~ -0.85 below) and hold8_low_acc (-0.85);
          std of a in -8 at v > 2.5 (the speed-hold residual; sigma_a_hold8 comes from
          fit_dynamics.py resid_std_by_mode 'hold8', 0.60)
standstill_v (0.05 m/s) is the smallest positive wheel reading (0.15 km/h = 0.042 m/s,
wheel_scale_noise.py); its open-loop effect is model_openloop.py --cfg no_latch (49 bags, H = 30 s: speed
bias +0.10 with the latch vs +0.34 m/s without, distance MAE 10.8 vs 13.9 m).

    python tools/calibration/standstill_hold8.py [--bags N|b1,b2] [--jobs 4]
"""
import argparse
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import _common as C

DT = 0.05
REST_V = 0.05
MOVE_V = 0.1


def runs(mask):
    """(start, end) index pairs of the True runs of a boolean array (end exclusive)."""
    m = np.r_[False, mask, False].astype(int)
    d = np.diff(m)
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def one(b):
    D = C.load_cache(b)
    if any(len((D.get(k) or {}).get('t_hdr', [])) < 100 for k in (C.FT, C.RT, C.CT)):
        return None
    K = C.params_for(b).k_for_vehicle()
    tf, vf = C.dedup_stream(D[C.FT])
    tr, vr = C.dedup_stream(D[C.RT])
    tc = np.sort(np.asarray(D[C.CT]['t_hdr'], float))
    T0 = max(tf[0], tr[0], tc[0])
    T1 = min(tf[-1], tr[-1], tc[-1])
    t = np.arange(T0, T1, DT)
    v = 0.5 * (np.interp(t, tf, vf) + np.interp(t, tr, vr)) / K
    a = (np.interp(t + 0.5, t, v) - np.interp(t - 0.5, t, v)) / 1.0
    n = C.notch_at(D, t)
    out = {'bag': b, 'starts': [], 'nocmd_starts': 0, 'onsets': 0, 'stops': [], 'm8': [], 'm8_va': None}
    # starts: first traction notch from rest (cmd stamps for the onset time)
    c = D[C.CT]
    o = np.argsort(c['t_hdr'], kind='stable')
    ct = np.asarray(c['t_hdr'], float)[o]
    cn = np.asarray(c['pos']).astype(int)[o]
    on = np.flatnonzero((cn[1:] > 0) & (cn[:-1] <= 0)) + 1
    for k in on:
        t_on = ct[k]
        if t_on - 2 < T0 or t_on + 10 > T1:
            continue
        w = (t >= t_on - 2) & (t <= t_on)
        if v[w].max() >= REST_V:
            continue
        after = np.flatnonzero((t > t_on) & (t <= t_on + 10) & (v > MOVE_V))
        if len(after):
            out['starts'].append(float(t[after[0]] - t_on))
    # motion onsets without a traction notch in the previous 5 s
    mv = np.flatnonzero((v[1:] > MOVE_V) & (v[:-1] <= MOVE_V)) + 1
    for i in mv:
        if i < 100 or v[i - 40:i].max() > REST_V + 0.05:
            continue
        out['onsets'] += 1
        if (n[i - 100:i] > 0).sum() == 0:
            out['nocmd_starts'] += 1
    # stops
    for s, e in runs(v < REST_V):
        if (e - s) * DT < 2 or s < 20:
            continue
        out['stops'].append((int(n[(s + e) // 2]), int(n[s - 1]), float(v[s - 1] - v[s - 21])))
    # -8 intervals
    for s, e in runs(n == -8):
        dur = (e - s) * DT
        out['m8'].append((dur, int(n[s - 1]) if s > 0 else 0, int(n[e]) if e < len(n) else 0,
                          float(v[e - 1]), float(v[s])))
    age = np.zeros(len(n))
    for s, e in runs(n == -8):
        age[s:e] = np.arange(e - s) * DT
    m = (n == -8) & (age > 1.0)
    out['m8_va'] = (v[m], a[m])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='', help="'' = all unique bags, N = first N, or a comma list")
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    bags = C.select_bags(a.bags, C.unique_bags())
    if a.jobs > 1:
        with ProcessPoolExecutor(a.jobs) as ex:
            res = [r for r in ex.map(one, bags) if r]
    else:
        res = [r for r in map(one, bags) if r]
    S = np.array([x for r in res for x in r['starts']])
    summ = {'n_bags': len(res)}
    print('%d bags' % len(res))
    if len(S):
        q = np.percentile(S, (10, 50, 90))
        print('start delay (traction onset from rest -> v > %.1f m/s): N=%d  p10 %.2f  median %.2f  p90 %.2f  min %.2f s   -> start_delay_s' % (
            MOVE_V, len(S), *q, S.min()))
        print('   share of starts faster than 1.2 s: %.1f%%' % (100 * np.mean(S < 1.2)))
        summ['start_delay'] = dict(n=len(S), p10=q[0], median=q[1], p90=q[2], min=float(S.min()), frac_below_1p2=float(np.mean(S < 1.2)))
    no = sum(r['nocmd_starts'] for r in res); on = sum(r['onsets'] for r in res)
    print('motion onsets from rest without a traction notch in the previous 5 s: %d of %d (%.1f%%)   -> hold8_rest_grow' % (
        no, on, 100.0 * no / max(on, 1)))
    summ['nocmd_starts'] = [no, on]
    st = [x for r in res for x in r['stops']]
    if st:
        nst = np.array([x[0] for x in st]); last = np.array([x[1] for x in st]); dv = np.array([x[2] for x in st])
        print('stops (v < %.2f for >= 2 s): N=%d, notch 0 during the stop %d (%.1f%%); last notch -14/-15 %.1f%%; dv over the last 1 s median %.2f m/s' % (
            REST_V, len(st), (nst == 0).sum(), 100 * np.mean(nst == 0), 100 * np.mean(last <= -14), np.median(dv)))
        summ['stops'] = dict(n=len(st), notch0=int((nst == 0).sum()), last_le_m14=float(np.mean(last <= -14)), dv_last_1s=float(np.median(dv)))
    m8 = [x for r in res for x in r['m8']]
    L = [x for x in m8 if x[0] > 5]
    if L:
        dur = np.array([x[0] for x in L]); ve = np.array([x[3] for x in L])
        print('-8 intervals > 5 s: N=%d, entered from -9 %d, exited to -9 %d; exit speed median %.2f (p10 %.2f p90 %.2f) m/s; duration p10/50/90 %.1f/%.1f/%.1f s' % (
            len(L), sum(x[1] == -9 for x in L), sum(x[2] == -9 for x in L), np.median(ve), *np.percentile(ve, (10, 90)),
            *np.percentile(dur, (10, 50, 90))))
        summ['m8_long'] = dict(n=len(L), exit_v_median=float(np.median(ve)), dur_median=float(np.median(dur)))
    V = np.concatenate([r['m8_va'][0] for r in res]); A = np.concatenate([r['m8_va'][1] for r in res])
    print('-8 (age > 1 s) wheel accel by speed   -> hold8_v / hold8_low_acc')
    bins = [0.3, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 8.0, 12.0, 20.0]
    summ['m8_bins'] = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (V >= lo) & (V < hi)
        if m.sum() < 20:
            continue
        print('   v %4.1f-%4.1f  N=%6d  mean a %+.3f  median %+.3f  std %.3f' % (lo, hi, m.sum(), A[m].mean(), np.median(A[m]), A[m].std()))
        summ['m8_bins'].append(dict(lo=lo, hi=hi, n=int(m.sum()), mean=float(A[m].mean()), median=float(np.median(A[m]))))
    for lab, m in (('v 0.3-2.5', (V > 0.3) & (V <= 2.5)), ('v > 2.5', V > 2.5)):
        if m.sum():
            print('   %-9s mean a %+.3f  std %.3f  (N=%d)' % (lab, A[m].mean(), A[m].std(), m.sum()))
            summ['m8_' + lab.replace(' ', '')] = dict(mean=float(A[m].mean()), std=float(A[m].std()), n=int(m.sum()))
    o = os.path.join(C.out_dir('standstill_hold8'), 'summary.json')
    C.save_json(o, summ)
    print('written', o)


if __name__ == '__main__':
    main()
