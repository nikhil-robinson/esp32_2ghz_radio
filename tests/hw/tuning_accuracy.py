#!/usr/bin/env python3
"""Absolute tuning accuracy over the whole range, with one board and no
transmitter: the receiver's own 40 MHz crystal harmonics are references at
exactly known frequencies. Each must appear at (harmonic - LO) from every LO,
including fractional LOs of the 6/5 divider band (1842..2209 MHz).

Usage: python3 tests/hw/tuning_accuracy.py /dev/ttyACM0
Result 2026-10-13: every reading within +-2 kHz.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'host'))
from esp_sdr import Device  # noqa: E402

TOLERANCE_KHZ = 5
SEARCH_KHZ = 150
MIN_SNR_DB = 12
# (harmonic MHz, LOs): divider band incl. fractional 1.2 x LO, and direct band.
CASES = ((1920, range(1914, 1927, 1)), (2160, range(2154, 2167, 1)),
         (2400, range(2397, 2407, 3)), (2520, range(2514, 2527, 3)), (2760, range(2754, 2767, 3)))
failures = []

with Device(sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0') as d:
    for harmonic, los in CASES:
        for lo in los:
            d.set_frequency(lo)
            iq, _ = d.capture(16380, 2)
            x = iq - iq.mean()
            X = np.abs(np.fft.fft(x * np.hanning(len(x)))) ** 2
            f = np.fft.fftfreq(len(x), 1 / 16e3)
            want = (harmonic - lo) * 1000
            m = np.abs(f - want) < SEARCH_KHZ
            k = np.flatnonzero(m)[np.argmax(X[m])]
            snr = 10 * np.log10(X[k] / np.median(X))
            ok = abs(f[k] - want) < TOLERANCE_KHZ and snr > MIN_SNR_DB
            print(f"{'PASS' if ok else 'FAIL'}  {harmonic} MHz harmonic from LO {lo}: "
                  f"{f[k] - want:+6.1f} kHz error ({snr:.0f} dB)")
            if not ok:
                failures.append((harmonic, lo))
print('ALL PASS' if not failures else f'{len(failures)} FAILED')
sys.exit(1 if failures else 0)
