#!/usr/bin/env python3
"""CPU and memory of the backup odometry node process, sampled with psutil (2 Hz).

The process is found by a substring of its command line (default
``backup_odometry_node``, the console script) or given by --pid.  A process whose
program is the match wins over one that only mentions it: under ``ros2 run`` the
wrapper's command line contains the node name too, and it has the lower pid.  Each sample:

* CPU in cores = (user + system CPU time delta) / wall delta (1.0 = one core
  fully busy; psutil ``cpu_percent`` / 100);
* RSS in MiB and the thread count.

Sampling stops after --duration, when the process exits, or on SIGINT; the
summary (mean / p50 / p95 / max CPU, RSS start / max) is printed and optionally
written as JSON, the raw samples as CSV.  ``--busy-only`` statistics restrict
CPU to samples where the process did any work (> 1% of a core), which excludes
the idle time before and after a bag play.

    python3 tools/resource_monitor.py [--pattern backup_odometry_node] [--duration S] [--csv f] [--json f]
"""
import argparse
import json
import math
import os
import sys
import time

import psutil


def is_program(argv, pattern):
    """The pattern names the program itself: argv[0], or argv[1] after a python interpreter.

    Distinguishes the node from wrappers that merely mention it, e.g.
    ``python3 /opt/ros/humble/bin/ros2 run tram_backup_odometry backup_odometry_node``
    (the parent of the node process under ``ros2 run``)."""
    progs = argv[:2] if argv and os.path.basename(argv[0]).startswith('python') else argv[:1]
    return any(pattern in os.path.basename(a) for a in progs)


def find_process(pattern, timeout):
    """The process whose program matches the pattern; else (after 2 s, the wrapper may still be
    starting the node) the first whose command line contains it."""
    me = os.getpid()
    t_end = time.monotonic() + timeout
    t_fallback = None
    while True:
        found = []
        for p in psutil.process_iter(['pid', 'cmdline']):
            argv = p.info['cmdline'] or []
            cmd = ' '.join(argv)
            if p.info['pid'] != me and pattern in cmd and 'resource_monitor' not in cmd:
                if is_program(argv, pattern):
                    return p
                found.append(p)
        now = time.monotonic()
        if found:
            t_fallback = now + 2.0 if t_fallback is None else t_fallback
            if now >= t_fallback or now > t_end:
                return found[0]
        elif now > t_end:
            return None
        time.sleep(0.2)


def pct(vals, q):
    if not vals:
        return float('nan')
    s = sorted(vals)
    i = q * (len(s) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def stats(vals):
    return {'n': len(vals), 'mean': sum(vals) / len(vals) if vals else float('nan'),
            'p50': pct(vals, 0.5), 'p95': pct(vals, 0.95), 'max': max(vals) if vals else float('nan')}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pattern', default='backup_odometry_node')
    ap.add_argument('--pid', type=int, default=None)
    ap.add_argument('--interval', type=float, default=0.5)
    ap.add_argument('--duration', type=float, default=0.0, help='0 = until the process exits / SIGINT')
    ap.add_argument('--wait', type=float, default=30.0, help='seconds to wait for the process to appear')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--json', default=None)
    a = ap.parse_args(argv)

    proc = psutil.Process(a.pid) if a.pid else find_process(a.pattern, a.wait)
    if proc is None:
        print('resource_monitor: no process matching %r' % a.pattern, file=sys.stderr)
        return 1
    cmd = ' '.join(proc.cmdline())
    rows = []
    t0 = time.monotonic()
    ct = proc.cpu_times()
    prev = (t0, ct.user + ct.system)
    cpu_start = prev[1]
    try:
        while not a.duration or time.monotonic() - t0 < a.duration:
            time.sleep(a.interval)
            try:
                with proc.oneshot():
                    ct = proc.cpu_times()
                    rss = proc.memory_info().rss / 2 ** 20
                    nth = proc.num_threads()
            except (psutil.NoSuchProcess, psutil.ZombieProcess):
                break
            now, cpu = time.monotonic(), ct.user + ct.system
            rows.append((now - t0, (cpu - prev[1]) / (now - prev[0]), rss, nth))
            prev = (now, cpu)
    except KeyboardInterrupt:
        pass

    cores = [r[1] for r in rows]
    rss = [r[2] for r in rows]
    busy = [c for c in cores if c > 0.01]
    wall = rows[-1][0] if rows else 0.0
    res = {
        'pid': proc.pid, 'cmdline': cmd, 'host_cpus': psutil.cpu_count(), 'samples': len(rows),
        'interval_s': a.interval, 'wall_s': wall,
        'cpu_s_total': prev[1] - cpu_start,
        'cpu_cores': stats(cores), 'cpu_cores_busy_only': stats(busy),
        'rss_mib': dict(stats(rss), start=rss[0] if rss else float('nan')),
        'threads_max': max((r[3] for r in rows), default=0),
    }
    c, b, m = res['cpu_cores'], res['cpu_cores_busy_only'], res['rss_mib']
    print('resource_monitor: pid %d, %d samples over %.1f s (host has %d CPUs)\n'
          '  CPU cores mean %.3f p50 %.3f p95 %.3f max %.3f | busy-only (%d samples) mean %.3f p95 %.3f\n'
          '  CPU time %.2f s | RSS MiB start %.1f mean %.1f max %.1f | threads %d'
          % (proc.pid, len(rows), wall, res['host_cpus'], c['mean'], c['p50'], c['p95'], c['max'],
             b['n'], b['mean'], b['p95'], res['cpu_s_total'], m['start'], m['mean'], m['max'],
             res['threads_max']), flush=True)
    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('t_s,cpu_cores,rss_mib,threads\n')
            f.writelines('%.3f,%.4f,%.2f,%d\n' % r for r in rows)
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
