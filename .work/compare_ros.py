"""Compare a run_checker.sh result bag with the offline pipeline and emulate the checker on it.

python3 compare_ros.py <result_bag_dir> [t_limit_s]
"""
import sys
import numpy as np

import os
ROOT = os.environ.get('SOL', '/Users/egor/Documents/sideprojects/приколы/ХакатонМосТранспорт/solution')
sys.path.insert(0, ROOT + '/tools')
import checker_sim as cs  # noqa: E402  (patches bagio for Odometry)
from tram_backup_odometry import bagio  # noqa: E402
from tram_backup_odometry.core.config import Params  # noqa: E402

REF = cs.REF
D = bagio.load_bag(cs.DEFAULT_BAG)
R = bagio.load_bag(sys.argv[1], topics=['/result/velocity', '/result/position', REF])
tlim = float(sys.argv[2]) if len(sys.argv) > 2 else None

p = Params(); p.vehicle_id = '30618'
pipe, rows, arr, cpu = cs.run(p, D)
off_v = {round(r.stamp, 6): r.v for r in rows}
off_p = {round(r.stamp, 6): (r.x, r.y, r.z) for r in rows if r.has_pose}

rv, rp = R['/result/velocity'], R['/result/position']
print('ROS outputs: velocity %d position %d | offline outputs %d (with pose %d)' % (
    len(rv['t_hdr']), len(rp['t_hdr']), len(rows), len(off_p)))
t0v = rv['t_hdr'][0]
dv = [(abs(v - off_v[round(t, 6)]), t) for t, v in zip(rv['t_hdr'], rv['v']) if round(t, 6) in off_v]
dp = [(float(np.linalg.norm(np.array([x, y, z]) - off_p[round(t, 6)])), t)
      for t, x, y, z in zip(rp['t_hdr'], rp['x'], rp['y'], rp['z']) if round(t, 6) in off_p]
print('velocity stamps in offline: %d/%d, max |dv| %.3g' % (len(dv), len(rv['t_hdr']), max(dv)[0] if dv else np.nan))
print('position stamps in offline: %d/%d, max |dp| %.3g m, p99 %.3g' % (
    len(dp), len(rp['t_hdr']), max(dp)[0] if dp else np.nan, np.percentile([a for a, _ in dp], 99) if dp else np.nan))
big = [(a, t - t0v) for a, t in dp if a > 0.05]
print('position |dp|>5 cm: %d, first at t=%s' % (len(big), '%.1f' % big[0][1] if big else '-'))

# checker emulation with the ACTUAL arrival order (record times of the result bag incl. the reference)
if REF in R:
    o = R[REF]
    def sc(label, o, res_t_rec, res_st, vals, kind):
        pr = cs.sync_pairs(o['t_rec'], o['t_hdr'], res_t_rec, res_st)
        if kind == 'v':
            e = np.array([vals[j] - o['vx'][i] for i, j in pr])
            print('%s: velocity RMSE %.6f max %.6f n %d' % (label, np.sqrt(np.mean(e ** 2)), np.abs(e).max(), len(e)))
        else:
            E = np.array([(vals[0][j] - o['x'][i], vals[1][j] - o['y'][i], vals[2][j] - o['z'][i]) for i, j in pr])
            d = np.linalg.norm(E, axis=1)
            print('%s: position x %.6f y %.6f z %.6f 3D RMSE %.6f max %.6f n %d' % (
                label, *np.sqrt(np.mean(E ** 2, axis=0)), np.sqrt(np.mean(d ** 2)), d.max(), len(d)))
    sc('emulated sync on recorded arrivals', o, rv['t_rec'], rv['t_hdr'], rv['v'], 'v')
    sc('emulated sync on recorded arrivals', o, rp['t_rec'], rp['t_hdr'], (rp['x'], rp['y'], rp['z']), 'p')
    print('reference msgs recorded: %d' % len(o['t_hdr']))

# where do they differ?
t0 = D['/vehicle/front_bogie_velocity']['t_hdr'][0]
um = np.array([t - t0 for t in rv['t_hdr'] if round(t, 6) not in off_v])
print('unmatched ROS velocity stamps per 10 s bucket:', np.histogram(um, bins=np.arange(0, 1320, 10))[0][:14].tolist() if len(um) else [])
offst = np.array(sorted(off_v)); rst = np.array(sorted(round(t, 6) for t in rv['t_hdr']))
lo, hi = rst[0], rst[-1]
om = [t - t0 for t in offst if lo <= t <= hi and t not in set(rst)]
print('offline stamps missing in ROS (within ROS span): %d; per 10 s:' % len(om), np.histogram(om, bins=np.arange(0, 1320, 10))[0][:14].tolist())
dpa = np.array([(t - t0, a) for a, t in dp])
for a in range(0, int(dpa[:, 0].max()) + 10, 10):
    m = (dpa[:, 0] >= a) & (dpa[:, 0] < a + 10)
    if m.any():
        print('  t %4d-%4d s: n %4d  max|dp| %.3f  mean %.3f' % (a, a + 10, m.sum(), dpa[m, 1].max(), dpa[m, 1].mean()))
dva = np.array([(t - t0, a) for a, t in dv])
print('|dv| ROS-offline same stamp: mean %.4f p99 %.4f; >0.05: %d' % (dva[:, 1].mean(), np.percentile(dva[:, 1], 99), (dva[:, 1] > 0.05).sum()))
for a in range(0, 1320, 60):
    m = (dva[:, 0] >= a) & (dva[:, 0] < a + 60); m2 = (dpa[:, 0] >= a) & (dpa[:, 0] < a + 60)
    if m.any():
        print('  min %4d: dv max %.3f rms %.4f | dp max %.3f rms %.3f' % (a, dva[m, 1].max(), np.sqrt(np.mean(dva[m, 1] ** 2)), dpa[m2, 1].max(), np.sqrt(np.mean(dpa[m2, 1] ** 2))))
# does the ordering explain it? velocity error vs reference for ROS outputs: stamp shared with offline vs not
o = D[REF]
import bisect
rt = o['t_hdr']
def verr(ts, vs):
    e = []
    for t, v in zip(ts, vs):
        i = np.searchsorted(rt, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(rt)), key=lambda k: abs(rt[k] - t))
        if abs(rt[j] - t) <= 0.05:
            e.append(v - o['vx'][j])
    return np.sqrt(np.mean(np.square(e))), len(e)
print('nearest-ref velocity RMSE: ROS all %.4f (n %d), offline all %.4f (n %d)' % (*verr(rv['t_hdr'], rv['v']), *verr([r.stamp for r in rows], [r.v for r in rows])))
