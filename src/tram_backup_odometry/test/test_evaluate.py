"""Sanity tests of the offline evaluator (tools/evaluate.py).

A synthetic reference drives along the real T2S pathgraph at 10 m/s; estimates
with known errors (along-track shift, lateral shift, speed bias, stamp shift,
missing poses) must produce exactly those metrics.  A real-data check (the
reference fed back as the estimate gives zero error) runs when the parsed bag
cache is present.  Needs numpy + scipy (tools dependency); skipped otherwise.

Runs under pytest or standalone:  python test_evaluate.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(HERE, '..'))

try:
    import numpy as np
    import evaluate as ev
except ImportError:  # scipy missing (ROS runtime image): nothing to test
    ev = None

V = 10.0
S0, S1 = 50.0, 2950.0


def _skip():
    if ev is None:
        try:
            import pytest
            pytest.skip('tools/evaluate.py needs scipy')
        except ImportError:
            pass
        return True
    return False


def _route_point(rt, s):
    return np.column_stack([np.interp(s, rt.s, rt.P[:, k]) for k in range(3)])


def _normal(rt, s):
    i = np.clip(np.searchsorted(rt.s, s) - 1, 0, len(rt.u) - 1)
    u = rt.u[i]
    return np.column_stack([-u[:, 1], u[:, 0]])  # left normal


def synthetic_ref():
    rt = ev.routes()['T2S']
    t = np.arange(0.0, (S1 - S0) / V, 0.1) + 1.0e9
    s = S0 + V * (t - t[0])
    xyz = _route_point(rt, s)
    xyz[:, 2] += 3.10
    odo = V * (t - t[0])
    return dict(tp=t, xyz=xyz, tv=t.copy(), v=np.full(len(t), V), dir='T2S', antenna='master',
                n_glitch={}, odo_t=t.copy(), odo=odo), s


def est_from(t, v, xyz, has_pose=None):
    return dict(t=t, v=v, x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
                has_pose=np.ones(len(t), bool) if has_pose is None else has_pose)


def test_exact_estimate_is_zero():
    if _skip():
        return
    ref, _ = synthetic_ref()
    R = ev.evaluate('syn', est_from(ref['tp'], ref['v'], ref['xyz']), ref=ref)
    assert R['v_cov'] == 1.0 and R['p_cov'] == 1.0
    for k in ('v_rmse', 'e3d_max', 'al_max', 'ct_max', 'drift_pct'):
        assert abs(R[k]) < 1e-9, (k, R[k])
    assert R['onmap_frac'] == 1.0 and R['score_loss'] < 1e-9


def test_along_track_shift():
    if _skip():
        return
    ref, s = synthetic_ref()
    rt = ev.routes()['T2S']
    xyz = _route_point(rt, s + 10.0)
    xyz[:, 2] += 3.10
    R = ev.evaluate('syn', est_from(ref['tp'], ref['v'], xyz), ref=ref)
    assert abs(R['al_mean'] - 10.0) < 0.01, R['al_mean']
    assert R['ct_rmse'] < 0.01, R['ct_rmse']
    assert 9.0 < R['e2d_mean'] <= 10.0 + 1e-6  # chord <= arc on curves
    # end drift: 10 m over the distance travelled up to the last sample
    assert abs(R['drift_pct'] - 100 * R['end_e2d'] / ref['odo'][-1]) < 1e-9


def test_lateral_shift():
    if _skip():
        return
    ref, s = synthetic_ref()
    xyz = ref['xyz'].copy()
    xyz[:, :2] += 1.0 * _normal(ev.routes()['T2S'], s)
    R = ev.evaluate('syn', est_from(ref['tp'], ref['v'], xyz), ref=ref)
    assert abs(R['ct_mean'] - 1.0) < 0.02, R['ct_mean']
    assert abs(R['al_mean']) < 0.05, R['al_mean']
    assert abs(R['e2d_mean'] - 1.0) < 1e-6


def test_speed_bias_and_regimes():
    if _skip():
        return
    ref, _ = synthetic_ref()
    R = ev.evaluate('syn', est_from(ref['tp'], ref['v'] + 0.1, ref['xyz']), ref=ref)
    assert abs(R['v_bias'] - 0.1) < 1e-9 and abs(R['v_rmse'] - 0.1) < 1e-9
    assert R['n_cruise'] == len(ref['tv'])
    t = np.arange(0.0, 30.0, 0.1)
    v = np.clip(np.where(t < 15, 0.5 * (t - 5), 5.0 - 0.5 * (t - 15)), 0, None)
    rg = ev.regimes(t, v)
    assert (rg[(t > 7) & (t < 13)] == 'accel').all()
    assert (rg[(t > 17) & (t < 23)] == 'brake').all()
    assert (rg[t < 4] == 'stopped').all()


def test_stamp_tolerance_and_missing_pose():
    if _skip():
        return
    ref, _ = synthetic_ref()
    # 1 Hz reference and estimate, so that only the shifted twin can match
    sparse = dict(ref, tp=ref['tp'][::10], xyz=ref['xyz'][::10], tv=ref['tv'][::10], v=ref['v'][::10])
    for shift, cov in ((0.04, 1.0), (0.06, 0.0)):
        R = ev.evaluate('syn', est_from(sparse['tp'] + shift, sparse['v'], sparse['xyz']), ref=sparse)
        assert R['v_cov'] == cov and R['p_cov'] == cov, (shift, R['v_cov'], R['p_cov'])
    hp = np.arange(len(ref['tp'])) % 2 == 0
    R = ev.evaluate('syn', est_from(ref['tp'], ref['v'], ref['xyz'], hp), ref=ref)
    assert abs(R['p_cov'] - 0.5) < 0.01 and R['v_cov'] == 1.0
    # duplicated stamps: the last published output wins
    t2 = np.r_[ref['tp'], ref['tp']]
    v2 = np.r_[ref['v'] + 5.0, ref['v']]
    R = ev.evaluate('syn', est_from(t2, v2, np.r_[ref['xyz'], ref['xyz']]), ref=ref)
    assert abs(R['v_rmse']) < 1e-9


def test_frozen_speed_mask():
    if _skip():
        return
    t = np.arange(0.0, 60.0, 0.1)
    # 5 m/s, braking at 1 m/s^2 over 25-30 s, then standing; GNSS speed frozen at 0 during 10-13 s
    v = np.clip(np.minimum(5.0, 30.0 - t), 0.0, None)
    x = np.r_[0.0, np.cumsum((v[1:] + v[:-1]) / 2 * 0.1)]
    v[(t >= 10) & (t < 13)] = 0.0
    xyz = np.column_stack([x, np.zeros_like(t), np.zeros_like(t)])
    xyz[(t > 40) & (t < 40.15), 0] += 2.0  # 2 m position jump while standing
    m = ev.frozen_speed_mask(t, v, t, xyz)
    assert m[(t >= 10.05) & (t < 12.95)].all()
    assert not m[(t >= 29.5)].any()  # real stop (incl. its first second) and the jump are kept


def test_projection_matches_package_route():
    if _skip():
        return
    from tram_backup_odometry.core.track_map import load_routes
    pk = load_routes(ev.MAPS_DIR)
    rng = np.random.default_rng(0)
    for name, rt in ev.routes().items():
        s = rng.uniform(5, rt.L - 5, 300)
        xy = _route_point(rt, s)[:, :2] + rng.normal(0, 2.0, (300, 2))
        se, de, le, _ = rt.project(xy)
        for k in range(len(xy)):
            sp, dp, lp, _ = pk[name].project(xy[k, 0], xy[k, 1])
            assert abs(sp - se[k]) < 0.6 and abs(dp - de[k]) < 0.05, (name, k, sp, se[k], dp, de[k])


def test_nonfinite_stamps_and_path_input(tmp_path=None):
    if _skip():
        return
    import pathlib
    import tempfile
    ref, _ = synthetic_ref()
    sparse = dict(ref, tp=ref['tp'][::10], xyz=ref['xyz'][::10], tv=ref['tv'][::10], v=ref['v'][::10])
    # estimate 0.03 s before every reference stamp plus a NaN-stamped output: all still match
    t = np.r_[sparse['tp'] - 0.03, np.nan]
    xyz = np.r_[sparse['xyz'], sparse['xyz'][:1]]
    R = ev.evaluate('syn', est_from(t, np.r_[sparse['v'], 0.0], xyz), ref=sparse)
    assert R['v_cov'] == 1.0 and R['p_cov'] == 1.0, (R['v_cov'], R['p_cov'])
    d = tmp_path or pathlib.Path(tempfile.mkdtemp())
    path = pathlib.Path(d) / 'est.npz'
    np.savez(str(path), **est_from(ref['tp'], ref['v'], ref['xyz']))
    R = ev.evaluate('syn', path, ref=ref)
    assert R['v_rmse'] == 0.0 and R['e3d_max'] == 0.0


def test_regimes_duplicate_stamp_and_missing_antenna():
    if _skip():
        return
    import warnings
    t = 1.786e9 + np.arange(0.0, 20.0, 0.1)
    v = 2.0 + 0.5 * (t - t[0])
    t[50] = t[49]  # duplicated header stamp on absolute unix time
    with warnings.catch_warnings():
        warnings.simplefilter('error')  # no division by a zero stamp spacing
        rg = ev.regimes(t, v)
    assert (rg[10:-10] == 'accel').all()
    # master RTK present, rover without RTK: no rover reference instead of an empty one
    n = 50
    fix = lambda st: dict(t_hdr=t[:n], t_rec=t[:n] + 0.05, lat=np.full(n, 55.8), lon=np.full(n, 37.4),
                          alt=np.full(n, 150.0), status=np.full(n, st))
    D = {ev.TOPIC_MFIX: fix(2), ev.TOPIC_RFIX: fix(0),
         ev.TOPIC_MVEL: dict(t_hdr=t[:n], t_rec=t[:n], vx=np.zeros(n), vy=np.zeros(n), vz=np.zeros(n))}
    assert ev.load_ref('syn', 'master', D=D) is not None
    assert ev.load_ref('syn', 'rover', D=D) is None and ev.load_ref('syn', 'mid', D=D) is None


def test_worst_list_puts_nan_last():
    if _skip():
        return
    import run_cv
    recs = [dict(bag='b%d' % i, status='ok', eval_ok=True, vehicle='30618', group='g', metrics={'master': m})
            for i, m in enumerate([dict(v_rmse=0.05, dir='T2S'), dict(v_rmse=np.nan, dir='T2S'),
                                   dict(v_rmse=0.2, dir='T2S')])]
    S = run_cv.summarize(recs, dict(antennas=['master'], tag='t'))
    assert [b for b, _ in S['worst']['v_rmse']] == ['b2', 'b0', 'b1']


def test_real_reference_roundtrip():
    if _skip() or not os.path.exists(ev.cache_path('30618_0e41eac3')):
        return
    ref = ev.load_ref('30618_0e41eac3')
    R = ev.evaluate('30618_0e41eac3', ev.reference_as_estimate(ref), ref=ref)
    assert R['v_cov'] == 1.0 and R['p_cov'] == 1.0
    assert R['v_rmse'] == 0.0 and R['e3d_max'] == 0.0 and R['al_max'] == 0.0
    assert R['dir'] == 'S2T'


if __name__ == '__main__':
    if ev is None:
        print('skipped: tools/evaluate.py needs numpy + scipy')
        sys.exit(0)
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print('ok', name)
