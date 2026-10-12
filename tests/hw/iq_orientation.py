#!/usr/bin/env python3
"""I/Q orientation on every tuning path, with a second board as a tone source.

Usage: python3 tests/hw/iq_orientation.py RX_PORT TX_PORT
RX_PORT runs firmware/. TX_PORT runs a tone source that understands
"TXON <mhz> <atten>", "TXF <field>", "TXOFF" (the phase-2 TX spike firmware
until TX lands in the library). Ground truth: a negative tone field is a
higher RF frequency (confirmed by a stock BLE receiver).
Result 2026-10-13: CORRECT on all paths (channel table, direct PLL, 6/5 divider).
"""
import sys, time, serial, numpy as np
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[2] / 'host'))
from esp_sdr import Device
tx = serial.Serial(sys.argv[2], 2000000, timeout=3); time.sleep(0.5); tx.reset_input_buffer()
def T(c):
    tx.write((c + '\n').encode()); return tx.readline().decode(errors='replace').strip()
def peak(dev):
    iq, _ = dev.capture(16380, 2)                       # 16 Msps
    x = iq - iq.mean(); X = np.abs(np.fft.fft(x * np.hanning(len(x))))**2
    f = np.fft.fftfreq(len(x), 1 / 16e6) / 1e3
    m = (np.abs(f) > 300) & (np.abs(f) < 3000)          # ignore DC / LO leakage
    k = np.flatnonzero(m)[np.argmax(X[m])]
    return f[k], 10 * np.log10(X[k] / np.median(X))
paths = [(2412, 'channel table'), (2437, 'channel table'), (2462, 'channel table'),
         (2402, 'direct PLL'), (2440, 'direct PLL'), (2480, 'direct PLL'), (2300, 'direct PLL'), (2600, 'direct PLL'),
         (2100, '6/5 divider'), (1900, '6/5 divider')]
with Device(sys.argv[1]) as rx:
    for mhz, path in paths:
        rx.set_frequency(mhz)
        res = []
        for field, true_khz in ((-6, +937), (+6, -937)):    # field<0 => higher RF (BLE-scanner proven)
            T(f'TXON {mhz} 0'); T(f'TXF {field}'); time.sleep(0.05)
            res.append((true_khz,) + peak(rx)); T('TXOFF')
        ok = all(snr > 20 for _, _, snr in res)
        verdict = ('no tone' if not ok else 'CORRECT' if all(np.sign(a) == np.sign(b) for a, b, _ in res)
                   else 'MIRRORED' if all(np.sign(a) == -np.sign(b) for a, b, _ in res) else 'unclear')
        print(f'{mhz} MHz {path:14s}: ' + '  '.join(f'true {t:+5d} -> seen {s:+7.0f} kHz ({n:.0f} dB)' for t, s, n in res) + f'  => {verdict}')
        if verdict != 'CORRECT':
            sys.exit(f'{mhz} MHz: {verdict}')
print('ALL PASS')
