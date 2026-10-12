#!/usr/bin/env python3
"""Transmitter over the air, two boards: one transmits, the other captures
16 Msps bursts that tools/demod.py (written from the datasheets, independent
of the C framing) decodes.

  carrier    frequency error at 2402 / 2440.5 / 2480 MHz
  fsk        deviation and sign (a run of 1 bits must sit above the carrier)
  nrf24      ShockBurst at 250k / 1M / 2M with CRC 0 / 1 / 2 bytes, plain and Gaussian
  ble        ADV_NONCONN_IND on 37 / 38 / 39 with a valid CRC-24

Usage: python3 tests/hw/tx_rf.py <tx port> <rx port>
The TX board is switched to transmit mode and back to receive at the end.
"""
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'host'))
sys.path.insert(0, str(ROOT / 'tools'))
import demod  # noqa: E402
from esp_sdr import Device  # noqa: E402

FS = 16e6
# Receiver LO 3 MHz below the carrier. Not 2: LOs that are multiples of 8 MHz
# (2400, 2424) sit on the receiver's own clock harmonics and cost BLE ch37/38
# 10-20 % of packets (measured).
IF_MHZ = 3
IF_HZ = IF_MHZ * 1e6
TONE_TOL_PPM = 50      # BLE and nRF24 receivers accept +-50 ppm
DEV_TOL = 0.10
MIN_RATE = 0.9         # decoded packets / packets that fit completely inside the captures
# demod.py is a plain discriminator with one slicer: weaker than a real GFSK
# receiver on isolated bits (Gaussian ISI at modulation index 0.32), so shaped
# packets are held to a lower bar. A real nRF24 is the judge for GFSK.
MIN_RATE_GFSK = 0.8
CAPTURE_S = 16380 / 16e6
BURST_S = 7.0          # each TX burst outlasts the captures (~5 s over USB)
CAPTURES = 40
RX_GAIN = 20
failures = []


def result(ok: bool, text: str):
    print(f"{'PASS' if ok else 'FAIL'}  {text}")
    if not ok:
        failures.append(text)


def capture(rx) -> np.ndarray:
    iq, _ = rx.capture(16380, 2)
    return iq - iq.mean()


def peak_offset_hz(iq: np.ndarray) -> float:
    spectrum = np.abs(np.fft.fft(iq * np.hanning(len(iq)))) ** 2
    return np.fft.fftfreq(len(iq), 1 / FS)[np.argmax(spectrum)] - IF_HZ


def receive_while(tx, rx, seq: int, decode, packet_s: float, period_s: float, label: str, bar: float = MIN_RATE):
    """Capture while a TX request runs; decoded packets must reach MIN_RATE of
    those that fit completely in the capture windows."""
    time.sleep(0.2)
    packets = sum(len(decode(capture(rx))) for _ in range(CAPTURES))
    tx.wait_tx(seq, 5 + 2 * BURST_S)
    expected = CAPTURES * (CAPTURE_S - packet_s) / period_s
    result(packets >= bar * expected, f'{label}: {packets} packets decoded, {expected:.0f} fit (bar {bar:.0%})')


def main(tx_port: str, rx_port: str):
    tx, rx = Device(tx_port), Device(rx_port)
    tx.set_mode(True)
    rx.set_mode(False)
    rx.set_gain(RX_GAIN)

    for khz in (2402000, 2440500, 2480000):
        rx.set_frequency(khz // 1000 - IF_MHZ)
        tx.tx_tone(khz)
        err = (peak_offset_hz(capture(rx)) - (khz % 1000) * 1e3) / 1e3
        tx.tx_off()
        result(abs(err) < TONE_TOL_PPM * khz / 1e6, f'carrier {khz} kHz: error {err:+.1f} kHz ({err / khz * 1e6:+.1f} ppm)')

    rx.set_frequency(2440 - IF_MHZ)
    for dev in (117188, 156250, 234375, 312500):  # 3, 4, 6, 8 fine steps
        seen = {}
        for bit in (1, 0):
            seq = tx.tx_bits([bit] * 256, 2440000, 250000, dev, count=1000, wait=False)
            time.sleep(0.2)
            seen[bit] = peak_offset_hz(capture(rx))
            tx.wait_tx(seq, 30)
        measured = (seen[1] - seen[0]) / 2
        result(abs(measured - dev) < DEV_TOL * dev,
               f'deviation {dev / 1e3:.1f} kHz: 1 -> {seen[1] / 1e3:+.1f}, 0 -> {seen[0] / 1e3:+.1f} kHz')

    addr = bytes([0xE7, 0xD3, 0xF0, 0x35, 0x77])
    payload = bytes(range(0xA0, 0xA8))
    channel = 40
    rx.set_frequency(2400 + channel - IF_MHZ)
    for bitrate, gap_us in ((250_000, 100), (1_000_000, 200), (2_000_000, 200)):
        for crc_len in (0, 1, 2):
            for gaussian in (False, True):
                bits = demod.nrf24_bits(addr, payload, crc_len)
                period = len(bits) / bitrate + gap_us * 1e-6
                count = min(65535, int(BURST_S / period))
                if gaussian:  # the shaped path goes through TX_BITS with the same packet
                    dev = 320000 if bitrate == 2_000_000 else 160000
                    seq = tx.tx_bits(bits, 2_400_000 + 1000 * channel, bitrate, dev, gap_us=gap_us,
                                     gaussian=True, wait=False, count=count)
                else:
                    seq = tx.tx_nrf24(channel, addr, payload, bitrate, crc_len, count=count, gap_us=gap_us,
                                      wait=False)
                good = lambda iq: [p for p in demod.nrf24_decode(demod.demod_bits(iq, FS, bitrate, IF_HZ), addr,  # noqa
                                                                 len(payload), crc_len, payload) if p['payload'] == payload]
                receive_while(tx, rx, seq, good, len(bits) / bitrate, period,
                              f"nrf24 {bitrate // 1000} kbps crc{crc_len} {'gfsk' if gaussian else 'fsk '}",
                              MIN_RATE_GFSK if gaussian else MIN_RATE)

    adv = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0xC6])
    data = bytes([2, 1, 6, 7, 9]) + b'ESPSDR'
    for channel, mhz in ((37, 2402), (38, 2426), (39, 2480)):
        rx.set_frequency(mhz - IF_MHZ)
        seq = tx.tx_ble(channel, adv, data, count=int(BURST_S / 420e-6), gap_us=200, wait=False)
        good = lambda iq: [p for p in demod.ble_decode(demod.demod_bits(iq, FS, 1e6, IF_HZ), channel)  # noqa
                           if p['adv_addr'] == adv and p['data'] == data]
        bits = len(demod.ble_adv_bits(channel, adv, data))
        receive_while(tx, rx, seq, good, bits * 1e-6, bits * 1e-6 + 200e-6, f'ble ch{channel}')

    tx.set_mode(False)
    print('ALL PASS' if not failures else f'{len(failures)} FAILED')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:3]))
