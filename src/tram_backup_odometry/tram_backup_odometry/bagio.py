"""Minimal rosbag2 (sqlite3 + CDR) reader for the tram topics, without ROS.

Used by the offline replay and the tools; the ROS node does not need it.
load_bag(path) -> {topic: {'t_rec','t_hdr','frame_id', <fields>}} with numpy arrays.
"""
import glob
import os
import sqlite3
import struct

import numpy as np


class _Cdr:
    def __init__(self, b):
        self.b = b
        self.o = 4  # skip encapsulation header

    def _al(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def i32(self):
        self._al(4); v = struct.unpack_from('<i', self.b, self.o)[0]; self.o += 4; return v

    def u32(self):
        self._al(4); v = struct.unpack_from('<I', self.b, self.o)[0]; self.o += 4; return v

    def u16(self):
        self._al(2); v = struct.unpack_from('<H', self.b, self.o)[0]; self.o += 2; return v

    def i8(self):
        v = struct.unpack_from('<b', self.b, self.o)[0]; self.o += 1; return v

    def u8(self):
        v = self.b[self.o]; self.o += 1; return v

    def f64(self):
        self._al(8); v = struct.unpack_from('<d', self.b, self.o)[0]; self.o += 8; return v

    def string(self):
        n = self.u32(); v = self.b[self.o:self.o + n - 1].decode('utf-8', 'replace'); self.o += n; return v

    def header(self):
        sec = self.i32(); ns = self.u32(); fid = self.string(); return sec + ns * 1e-9, fid


def _parse(typ, b):
    r = _Cdr(b)
    th, fid = r.header()
    if typ.endswith('VelocitySensor'):
        return th, fid, {'v': r.f64()}
    if typ.endswith('DriverControllerCommand'):
        return th, fid, {'pos': r.i8()}
    if typ.endswith('NavSatFix'):
        st = r.i8(); sv = r.u16(); lat = r.f64(); lon = r.f64(); alt = r.f64()
        cov = [r.f64() for _ in range(9)]; ct = r.u8()
        return th, fid, {'lat': lat, 'lon': lon, 'alt': alt, 'status': st, 'service': sv, 'cov': cov, 'cov_type': ct}
    if typ.endswith('TwistStamped'):
        vals = [r.f64() for _ in range(6)]
        return th, fid, dict(zip(['vx', 'vy', 'vz', 'wx', 'wy', 'wz'], vals))
    return None


def db3_path(path):
    if path.endswith('.db3'):
        return path
    files = sorted(glob.glob(os.path.join(path, '*.db3')))
    if not files:
        raise FileNotFoundError('no .db3 in %s' % path)
    return files[0]


def load_bag(path, topics=None):
    con = sqlite3.connect(db3_path(path))
    try:
        tmap = {i: (n, t) for i, n, t in con.execute('select id,name,type from topics')}
        out = {}
        for tid, (name, typ) in tmap.items():
            if topics is not None and name not in topics:
                continue
            acc = {'t_rec': [], 't_hdr': [], 'frame_id': []}
            rows = con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,))
            for ts, data in rows:
                parsed = _parse(typ, bytes(data))
                if parsed is None:
                    continue
                th, fid, f = parsed
                acc['t_rec'].append(ts * 1e-9); acc['t_hdr'].append(th); acc['frame_id'].append(fid)
                for k, v in f.items():
                    acc.setdefault(k, []).append(v)
            out[name] = {k: (np.array(v) if k != 'frame_id' else v) for k, v in acc.items()}
        return out
    finally:
        con.close()
