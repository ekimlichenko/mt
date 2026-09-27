"""Shared paths and helpers of the offline calibration scripts (tools/calibration/*).

All paths are relative to the solution root (the parent of tools/):
    cache    .work/cache/<bag>.pkl        (tools/build_cache.py; topic -> dict of numpy arrays)
    index    .work/bag_index.json         (tools/bag_index.py; eval_ok, rtk_frac, fold, ...)
    dataset  ../dataset/data/<bag>/       (metadata.yaml only: the vehicle day of a bag)
    maps     src/tram_backup_odometry/maps/{t2s,s2t}.json
    package  src/tram_backup_odometry     (added to sys.path; the scripts only import core/*)
Outputs go to $CALIB_WORK/<step>/ (default .work/calibration/<step>/).
"""
import hashlib
import json
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PKG_DIR = os.path.join(ROOT, 'src', 'tram_backup_odometry')
CACHE = os.path.join(ROOT, '.work', 'cache')
INDEX = os.path.join(ROOT, '.work', 'bag_index.json')
DATASET = os.path.join(os.path.dirname(ROOT), 'dataset', 'data')
MAPS = os.path.join(PKG_DIR, 'maps')
WORK = os.path.abspath(os.environ.get('CALIB_WORK', os.path.join(ROOT, '.work', 'calibration')))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
TOOLS = os.path.dirname(HERE)
if TOOLS not in sys.path:
    sys.path.append(TOOLS)           # tools/evaluate.py: judge-like GNSS reference (load_ref)

from tram_backup_odometry.core.config import Params                        # noqa: E402
from tram_backup_odometry.core.frames import MAP_OFFSET_E, MAP_OFFSET_N    # noqa: E402

FT = '/vehicle/front_bogie_velocity'
RT = '/vehicle/rear_bogie_velocity'
CT = '/vehicle/driver_position_cmd'
MF = '/sensing/gnss/master/fix'
MV = '/sensing/gnss/master/vel'
RF = '/sensing/gnss/rover/fix'
RV = '/sensing/gnss/rover/vel'


def out_dir(step):
    d = os.path.join(WORK, step)
    os.makedirs(d, exist_ok=True)
    return d


def load_cache(bag):
    with open(os.path.join(CACHE, bag + '.pkl'), 'rb') as f:
        return pickle.load(f)


def cache_bags():
    return sorted(f[:-4] for f in os.listdir(CACHE) if f.endswith('.pkl'))


def unique_bags():
    """Unique cached bags: dup_of is None in .work/bag_index.json, else md5 of the front wheel values."""
    if os.path.exists(INDEX):
        idx = load_index()['bags']
        return sorted(b for b, v in idx.items() if not v.get('dup_of'))
    seen, out = set(), []
    for b in cache_bags():
        d = load_cache(b)
        h = hashlib.md5(np.asarray(d.get(FT, {}).get('v', np.array([]))).tobytes()).hexdigest()
        if h in seen:
            continue
        seen.add(h)
        out.append(b)
    return out


def load_index():
    with open(INDEX) as f:
        return json.load(f)


def select_bags(spec, default):
    """--bags: '' -> default list; 'N' -> first N of default; 'eval' -> eval_ok of the index;
    otherwise a comma list of bag names."""
    if not spec:
        return list(default)
    if spec.isdigit():
        return list(default)[:int(spec)]
    if spec == 'eval':
        idx = load_index()['bags']
        return sorted(b for b, v in idx.items() if v.get('eval_ok'))
    return [b.strip() for b in spec.split(',') if b.strip()]


def vehicle_of(bag):
    return bag.split('_')[0]


def params_for(bag=None, sets=(), **kw):
    """Params() with the vehicle of the bag, keyword overrides and 'key=value' strings."""
    p = Params()
    if bag is not None:
        p.vehicle_id = vehicle_of(bag)
    for k, v in kw.items():
        setattr(p, k, v)
    set_params(p, sets)
    return p


def _cast(old, v):
    if isinstance(old, bool):
        return v.lower() in ('1', 'true', 'yes', 'on')
    if isinstance(old, int):
        return int(float(v))
    if isinstance(old, float):
        return float(v)
    if isinstance(old, str):
        return v
    try:
        return float(v)
    except ValueError:
        return v


def set_params(p, sets):
    for kv in sets or ():
        k, v = kv.split('=', 1)
        setattr(p, k, _cast(getattr(p, k, None), v))
    return p


def parse_sets(sets):
    """['k=v', ...] -> dict with values cast like Params fields."""
    ref = Params()
    out = {}
    for kv in sets or ():
        k, v = kv.split('=', 1)
        out[k] = _cast(getattr(ref, k, None), v)
    return out


def utm_vec(lat, lon, lon0=39.0, k0=0.9996, fe=500000.0):
    """Vectorised core.frames.utm (Krueger series) -> map frame (UTM 37N minus the map offset)."""
    f = 1 / 298.257223563
    a = 6378137.0
    n = f / (2 - f)
    A = a / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    al = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16, 13 * n ** 2 / 48 - 3 * n ** 3 / 5, 61 * n ** 3 / 240)
    e = math.sqrt(f * (2 - f))
    phi = np.radians(np.asarray(lat, float))
    lam = np.radians(np.asarray(lon, float) - lon0)
    t = np.sinh(np.arctanh(np.sin(phi)) - e * np.arctanh(e * np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1 + t * t))
    E = eta + sum(al[j - 1] * np.cos(2 * j * xi) * np.sinh(2 * j * eta) for j in (1, 2, 3))
    N = xi + sum(al[j - 1] * np.sin(2 * j * xi) * np.cosh(2 * j * eta) for j in (1, 2, 3))
    return fe + k0 * A * E - MAP_OFFSET_E, k0 * A * N - MAP_OFFSET_N


def nearest(tq, ts):
    """Index of the nearest ts (sorted) for every tq and the time distance."""
    tq = np.asarray(tq, float)
    j = np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1)
    jl = j - 1
    pick = np.where(np.abs(ts[jl] - tq) <= np.abs(ts[j] - tq), jl, j)
    return pick, np.abs(ts[pick] - tq)


def gnss_speed_ref(D, tol=0.05):
    """(t, v, vx, vy) of master vel samples with a status-2 master fix within tol, sorted by t (or None)."""
    mv = D.get(MV) or {}
    mf = D.get(MF) or {}
    if len(mv.get('t_hdr', [])) == 0 or len(mf.get('t_hdr', [])) == 0:
        return None
    tv = np.asarray(mv['t_hdr'], float)
    vx = np.asarray(mv['vx'], float)
    vy = np.asarray(mv['vy'], float)
    tf = np.asarray(mf['t_hdr'], float)
    st = np.asarray(mf['status'])
    o = np.argsort(tf, kind='stable')
    tf, st = tf[o], st[o]
    j, dt = nearest(tv, tf)
    ok = (dt <= tol) & (st[j] == 2)
    tv, vx, vy = tv[ok], vx[ok], vy[ok]
    o = np.argsort(tv, kind='stable')
    return tv[o], np.hypot(vx, vy)[o], vx[o], vy[o]


def dedup_stream(x, key='v', min_dt=0.02):
    """Header-sorted samples of a wheel stream with repeated stamps (dt <= min_dt) removed."""
    t = np.asarray(x['t_hdr'], float)
    v = np.asarray(x[key], float)
    o = np.argsort(t, kind='stable')
    t, v = t[o], v[o]
    keep = np.r_[True, np.diff(t) > min_dt]
    return t[keep], v[keep]


def notch_at(D, t):
    """Driver notch in force at times t (zero-order hold on header stamps; 0 before the first cmd)."""
    c = D[CT]
    tc = np.asarray(c['t_hdr'], float)
    n = np.asarray(c['pos']).astype(int)
    o = np.argsort(tc, kind='stable')
    tc, n = tc[o], n[o]
    i = np.searchsorted(tc, t, 'right') - 1
    return np.where(i >= 0, n[np.clip(i, 0, None)], 0)


def save_json(path, obj):
    def conv(x):
        if isinstance(x, (np.floating,)):
            return float(x)
        if isinstance(x, (np.integer,)):
            return int(x)
        if isinstance(x, np.ndarray):
            return x.tolist()
        raise TypeError(type(x))
    with open(path, 'w') as f:
        json.dump(obj, f, indent=1, default=conv)
