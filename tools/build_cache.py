#!/usr/bin/env python3
"""Parse the recorded rosbag2 bags into the offline cache ``.work/cache/<bag>.pkl``.

Every offline tool (``run_cv.py``, ``evaluate.py``, ``bag_index.py``, ``plots.py``)
reads a bag from this cache instead of the sqlite file: one pickle per bag with
``{topic: {'t_rec', 't_hdr', 'frame_id', <message fields>}}`` (numpy arrays; the
format of ``tram_backup_odometry.bagio.load_bag``, which parses the CDR payload
without ROS).  Parsing all 122 bags takes a few minutes; afterwards a replay of
one bag needs ~1-3 s.

A bag is any directory under ``--data`` that holds a ``metadata.yaml`` or a
``*.db3`` file; the cache file is named after the directory.  Existing cache
files are kept unless ``--force``; files are written atomically (temporary file
+ rename), so an interrupted run never leaves a truncated pickle behind.

    python tools/build_cache.py                         # all bags of ../dataset/data -> .work/cache/
    python tools/build_cache.py 30618_0e41eac3 0e41eac3 # selected bags (full name or unique suffix)
    python tools/build_cache.py --out /tmp/cache --jobs 4 --force
    python tools/build_cache.py --list                  # show what would be done

After (re)building the cache, rebuild the bag index: ``python tools/bag_index.py``.
"""
import argparse
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_DIR = os.path.join(ROOT, 'src', 'tram_backup_odometry')
DEFAULT_DATA = os.path.join(os.path.dirname(ROOT), 'dataset', 'data')
DEFAULT_OUT = os.path.join(ROOT, '.work', 'cache')
PICKLE_PROTOCOL = 4


def list_bags(data_dir):
    """Sorted bag directory names under data_dir (a directory with metadata.yaml or *.db3)."""
    out = []
    for name in sorted(os.listdir(data_dir)):
        d = os.path.join(data_dir, name)
        if not os.path.isdir(d):
            continue
        files = os.listdir(d)
        if 'metadata.yaml' in files or any(f.endswith('.db3') for f in files):
            out.append(name)
    return out


def select(all_bags, wanted):
    """Resolve full names or unique suffixes (e.g. the hash part) to bag names."""
    if not wanted:
        return list(all_bags)
    sel = []
    for w in wanted:
        hits = [b for b in all_bags if b == w] or [b for b in all_bags if b.endswith(w)]
        if len(hits) != 1:
            raise SystemExit('bag %r: %s' % (w, 'not found' if not hits else 'ambiguous (%s)' % ', '.join(hits)))
        if hits[0] not in sel:
            sel.append(hits[0])
    return sel


def convert(bag, data_dir, out_dir, force):
    """Parse one bag and write <out_dir>/<bag>.pkl; returns (bag, status, seconds, n_messages)."""
    if PKG_DIR not in sys.path:          # worker processes (spawn) start with a fresh sys.path
        sys.path.insert(0, PKG_DIR)
    from tram_backup_odometry.bagio import load_bag

    dst = os.path.join(out_dir, bag + '.pkl')
    if os.path.exists(dst) and not force:
        return bag, 'skip', 0.0, 0
    t0 = time.time()
    D = load_bag(os.path.join(data_dir, bag))
    n = sum(len(v['t_rec']) for v in D.values())
    tmp = dst + '.tmp%d' % os.getpid()
    try:
        with open(tmp, 'wb') as f:
            pickle.dump(D, f, protocol=PICKLE_PROTOCOL)
        os.replace(tmp, dst)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    return bag, 'ok', time.time() - t0, n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*', help='bag names or unique suffixes (default: all bags in --data)')
    ap.add_argument('--data', default=DEFAULT_DATA, help='directory with the bag directories (default %(default)s)')
    ap.add_argument('--out', default=DEFAULT_OUT, help='cache directory (default %(default)s)')
    ap.add_argument('--jobs', type=int, default=max(1, min(8, (os.cpu_count() or 2) - 2)))
    ap.add_argument('--force', action='store_true', help='rewrite existing cache files')
    ap.add_argument('--list', action='store_true', help='only list the bags and whether they are cached')
    a = ap.parse_args(argv)

    if not os.path.isdir(a.data):
        ap.error('data directory not found: %s (unpack dataset/data.zip or pass --data)' % a.data)
    bags = select(list_bags(a.data), a.bags)
    if not bags:
        ap.error('no bags found in %s' % a.data)
    os.makedirs(a.out, exist_ok=True)
    if a.list:
        for b in bags:
            print('%-16s %s' % (b, 'cached' if os.path.exists(os.path.join(a.out, b + '.pkl')) else '-'))
        return 0

    print('%d bags: %s -> %s (jobs %d%s)' % (len(bags), a.data, a.out, a.jobs, ', force' if a.force else ''))
    t0 = time.time()
    counts = {'ok': 0, 'skip': 0, 'error': 0}
    with ProcessPoolExecutor(a.jobs) as ex:
        futs = {ex.submit(convert, b, a.data, a.out, a.force): b for b in bags}
        for i, f in enumerate(as_completed(futs), 1):
            b = futs[f]
            try:
                _, status, dt, n = f.result()
                msg = '%d messages, %.1f s' % (n, dt) if status == 'ok' else 'exists (use --force)'
            except Exception as e:     # one broken bag must not stop the others
                status, msg = 'error', '%s: %s' % (e.__class__.__name__, e)
            counts[status] += 1
            print('[%3d/%d] %-16s %-5s %s' % (i, len(bags), b, status, msg), flush=True)
    print('done in %.0f s: %d written, %d skipped, %d errors -> %s'
          % (time.time() - t0, counts['ok'], counts['skip'], counts['error'], a.out))
    return 1 if counts['error'] else 0


if __name__ == '__main__':
    sys.exit(main())
