#!/usr/bin/env python3
"""Measure a board's crystal error against a carrier of known frequency.

    python3 tools/calibrate.py <rx port> <reference kHz> [--tx <tx port>]

The reference must be accurate (signal generator, GPS-disciplined source):
a board calibrated against another uncalibrated board is only as good as
that board. Prints the ppb to pass to esp_sdr_set_crystal_ppb() /
Device.set_crystal_ppb(). With --tx, the receiver is then calibrated and
measures the transmitting board's carrier at the same frequency.

Transmitting heats the chip (up to ~5 ppm/min for a continuous carrier), so
calibrate a transmitter at the duty cycle it will run.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
from esp_sdr import Device  # noqa: E402

FS = 16e6
IF_MHZ = 3  # LO off the receiver's own n x 8 MHz clock harmonics
SEARCH_HZ = 150e3  # +-60 ppm at 2.5 GHz


def carrier_offset_hz(rx: Device, khz: int, averages: int = 8) -> float:
    """Measured minus expected carrier frequency, as the receiver sees it."""
    lo = khz // 1000 - IF_MHZ
    expected = (khz - lo * 1000) * 1e3
    readings = []
    for _ in range(averages):
        iq, _ = rx.capture(16380, 2)
        x = iq - iq.mean()
        X = np.abs(np.fft.fft(x * np.hanning(len(x)))) ** 2
        f = np.fft.fftfreq(len(x), 1 / FS)
        m = np.abs(f - expected) < SEARCH_HZ
        k = np.flatnonzero(m)[np.argmax(X[m])]
        a, b, c = np.log(X[k - 1]), np.log(X[k]), np.log(X[k + 1])
        readings.append(f[k] + 0.5 * (a - c) / (a - 2 * b + c) * FS / len(x) - expected)  # parabolic peak
    return float(np.median(readings))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('rx')
    ap.add_argument('khz', type=int)
    ap.add_argument('--tx', help='also calibrate this board as a transmitter')
    a = ap.parse_args()

    rx = Device(a.rx)
    rx.set_mode(False)
    rx.set_crystal_ppb(0)
    rx.set_frequency(a.khz // 1000 - IF_MHZ)
    if a.tx:
        tx = Device(a.tx)
        tx.set_mode(True)
        tx.set_crystal_ppb(0)
        tx.tx_off()
    # A receiver running fast tunes high, so the reference appears low.
    rx_ppb = round(-carrier_offset_hz(rx, a.khz) / (a.khz * 1e3) * 1e9)
    print(f'receiver {a.rx}: {rx_ppb:+d} ppb')
    if a.tx:
        input('Switch the reference off, then press Enter to measure the transmitter... ')
        rx.set_crystal_ppb(rx_ppb)
        rx.set_frequency(a.khz // 1000 - IF_MHZ)
        tx.tx_tone(a.khz)
        time.sleep(0.05)
        tx_ppb = round(carrier_offset_hz(rx, a.khz, 4) / (a.khz * 1e3) * 1e9)
        tx.tx_off()
        print(f'transmitter {a.tx}: {tx_ppb:+d} ppb')


if __name__ == '__main__':
    main()
