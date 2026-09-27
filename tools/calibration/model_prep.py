"""Builds the per-bag arrays of the model / estimator validation into $CALIB_WORK/model_prep/
(see model_common.py; ~49 clean RTK bags).  model_openloop.py / model_closedloop.py build missing
bags on demand, so this step only parallelises it.

    python tools/calibration/model_prep.py [--bags N|b1,b2] [--jobs 4]
"""
import argparse
from concurrent.futures import ProcessPoolExecutor

import model_common as M


def one(b):
    d = M.load(b)
    return b, d['direction'], d['direction_index'], len(d['S']['t']), float(d['vel']['gate'].mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='')
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    B = M.bag_list(a.bags)
    print(len(B), 'bags ->', M.PREP)
    with ProcessPoolExecutor(max(a.jobs, 1)) as ex:
        for b, dr, di, ns, g in ex.map(one, B):
            print('%-16s %s (index %s)  RTK on-route fixes %6d  gated vel %.3f' % (b, dr, di, ns, g), flush=True)


if __name__ == '__main__':
    main()
