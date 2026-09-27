"""Timing offset between the wheel speed (header stamps) and the gated GNSS master speed: the
shift L minimising sum (wheel(t + L) - gnss(t))^2 over the clean RTK bags (model_common.py).
Supports that the wheel header stamps need no latency correction (analysis_notes C*: the
measurement is used at its header stamp; Params time_* handle only glitches / late messages).
Source: .work/scratch_model/lagcheck.py.  Result (17 bags, every 3rd): L = 0.00 s with a sharp
minimum (SSE +12 % / +9 % at -0.02 / +0.02 s, x3.6-3.8 at -+0.10 s), i.e. no systematic lag
between the wheel header stamps and the GNSS header stamps.

    python tools/calibration/model_lagcheck.py [--bags N|b1,b2] [--every 3]
Light: ~20 s on every 3rd of the 49 bags (prepared arrays, model_prep.py builds missing ones).
"""
import argparse

import numpy as np

import model_common as M


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bags', default='')
    ap.add_argument('--every', type=int, default=3, help='use every k-th bag')
    a = ap.parse_args()
    lags = np.round(np.arange(-0.30, 0.301, 0.02), 2)
    acc = np.zeros(len(lags))
    n = 0
    for b in M.bag_list(a.bags)[::a.every]:
        d = M.load(b)
        m = d['vel']['gate']
        tg, vg = d['vel']['t'][m], d['vel']['v'][m]
        o = np.argsort(d['F']['t'])
        tF, vF = d['F']['t'][o], d['F']['v'][o]
        ok = (tg > tF[0] + 1) & (tg < tF[-1] - 1)
        tg, vg = tg[ok], vg[ok]
        for i, L in enumerate(lags):
            e = np.interp(tg + L, tF, vF) - vg          # wheel at t+L compared with GNSS at t
            e = e[np.abs(e) < 0.5]
            acc[i] += np.sum(e * e)
        n += 1
    best = lags[np.argmin(acc)]
    print('%d bags; SSE-optimal wheel shift (wheel(t+L) ~ gnss(t)): L = %+.2f s' % (n, best))
    for L, s in zip(lags, acc):
        print('  %+.2f  %.4f' % (L, s / acc.min()))


if __name__ == '__main__':
    main()
