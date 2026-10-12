#!/usr/bin/env python3
"""Absolute frequency orientation of every output path, with no assumption
about the transmitter's modulation: a plain carrier at a fixed frequency, and
the receiver's LO moved around it. A correct receiver sees it at (carrier - LO).

Usage: python3 tests/hw/iq_orientation.py RX_PORT TX_PORT
RX_PORT runs firmware/; TX_PORT runs a carrier source understanding
"TXON <mhz> <atten>", "TXF 0", "TXOFF" (phase-2 TX spike firmware).
"""
import sys
import time
from pathlib import Path

import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'host'))
from esp_sdr import Device  # noqa: E402

CARRIER_MHZ = 2445
TOLERANCE_KHZ = 200  # board-to-board crystal error is ~100 kHz at 2.4 GHz
failures = []


def check(name, seen_khz, want_khz):
    ok = abs(seen_khz - want_khz) < TOLERANCE_KHZ
    print(f"{'PASS' if ok else 'FAIL'}  {name}: carrier at {seen_khz:+8.1f} kHz, want {want_khz:+8.1f}")
    if not ok:
        failures.append(name)


def strongest(freqs_khz, power, exclude_khz=20):
    m = np.abs(freqs_khz) > exclude_khz  # skip DC / LO leakage
    return freqs_khz[np.flatnonzero(m)[np.argmax(power[m])]]


tx = serial.Serial(sys.argv[2], 2000000, timeout=3)
time.sleep(0.3)
tx.reset_input_buffer()
for c in (f'TXON {CARRIER_MHZ} 0', 'TXF 0'):
    tx.write((c + '\n').encode())
    tx.readline()
try:
    with Device(sys.argv[1]) as d:
        for lo in range(CARRIER_MHZ - 4, CARRIER_MHZ + 5, 2):
            d.set_frequency(lo)
            iq, _ = d.capture(16380, 2)
            x = iq - iq.mean()
            X = np.abs(np.fft.fft(x * np.hanning(len(x)))) ** 2
            check(f'burst capture, LO {lo} MHz', strongest(np.fft.fftfreq(len(x), 1 / 16e3), X),
                  (CARRIER_MHZ - lo) * 1000)

        d.set_frequency(CARRIER_MHZ - 2)
        frames = [f for f in d.stream('SPECTRUM', rate=2, nfft=1024, duration_ms=300)
                  if f.message.name == 'SPECTRUM_FRAME']
        bins = np.mean([np.frombuffer(f.body(), np.uint8) for f in frames], axis=0)
        check('spectrum stream, LO -2 MHz', strongest(np.fft.fftfreq(1024, 1 / 16e3), bins), 2000)

        d.request('SET_PLL_OFFSET', khz=50)
        d.set_frequency(CARRIER_MHZ)
        frames = [f for f in d.stream('IQ_STREAM', rate=2, decimation=64, bits=8, shift=255, duration_ms=500)
                  if f.message.name == 'IQ_FRAME']
        v = np.frombuffer(b''.join(f.body() for f in frames), np.int8).astype(np.float32)
        iq = v[0::2] + 1j * v[1::2]  # IQ stream output is already corrected
        x = iq[-32768:] - iq[-32768:].mean()
        X = np.abs(np.fft.fft(x * np.hanning(len(x)))) ** 2
        check('IQ stream (250 kS/s), LO +50 kHz', strongest(np.fft.fftfreq(len(x), 1 / 250.0), X, 5), -50)
        d.request('SET_PLL_OFFSET', khz=0)
finally:
    tx.write(b'TXOFF\n')

print('ALL PASS' if not failures else f'{len(failures)} FAILED')
sys.exit(1 if failures else 0)
