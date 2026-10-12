#!/usr/bin/env python3
"""Sweep the whole receive range and save a CSV (and a PNG if matplotlib is installed).

Usage: python3 tools/scan.py /dev/ttyACM0 [--rate 1] [--from 1842] [--to 2790]
                              [--nfft 1024] [--averages 8] [--gain 40] [--out scan]
rate 0/1/2 = 80/40/16 Msps capture per step (32/16/6 MHz steps).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
from esp_sdr import Device  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port')
    ap.add_argument('--rate', type=int, default=1, choices=(0, 1, 2))
    ap.add_argument('--from', dest='start', type=int, default=None, help='MHz (default: lowest allowed)')
    ap.add_argument('--to', type=int, default=2790)
    ap.add_argument('--nfft', type=int, default=1024)
    ap.add_argument('--averages', type=int, default=8)
    ap.add_argument('--gain', type=int, default=40, help='PHY gain index; -1 = AGC')
    ap.add_argument('--out', default='scan')
    a = ap.parse_args()
    start = a.start or 1840 + {0: 4, 1: 2, 2: 1}[a.rate]
    with Device(a.port) as dev:
        f, db, run = dev.scan(start, a.to, a.rate, a.nfft, a.averages, None if a.gain < 0 else a.gain)
    np.savetxt(f'{a.out}.csv', np.column_stack([f, db]), fmt=['%.0f', '%.1f'], delimiter=',',
               header='frequency_hz,power_db_fs')
    print(f'{len(f)} bins, {f[0] / 1e6:.2f}-{f[-1] / 1e6:.2f} MHz -> {a.out}.csv')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.plot(f / 1e6, db, lw=0.4)
    ax.set_xlabel('Frequency (MHz)')
    ax.set_ylabel('Power (dB, relative to ADC full scale)')
    ax.set_title(f'ESP32-S3 sweep {f[0] / 1e6:.0f}-{f[-1] / 1e6:.0f} MHz, '
                 f'{(f[1] - f[0]) / 1e3:.1f} kHz bins, gain {a.gain if a.gain >= 0 else "AGC"}')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{a.out}.png', dpi=130)
    print(f'-> {a.out}.png')


if __name__ == '__main__':
    main()
