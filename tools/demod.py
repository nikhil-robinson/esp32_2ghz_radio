#!/usr/bin/env python3
"""2-FSK/GFSK packet decoder for captured IQ, written from the specs
(BLE Core Vol 6 Part B 2.1, 3.1-3.2; nRF24L01+ datasheet 7.3) and deliberately
independent of the C framing in components/esp_sdr/src/tx/packet.c.

    bits = demod_bits(iq, fs=16e6, bitrate=1e6, offset_hz=2e6)
    ble_decode(bits, channel=37)          -> dict or None
    nrf24_decode(bits, addr, n, crc_len)  -> dict or None

Self-test: python3 tools/demod.py
"""
from __future__ import annotations

import numpy as np

BLE_AA = 0x8E89BED6


# ---- reference framing (bit lists, air order) ---------------------------

def _lsb_bits(data: bytes) -> list[int]:
    return [(b >> i) & 1 for b in data for i in range(8)]


def _msb_bits(data: bytes) -> list[int]:
    return [(b >> (7 - i)) & 1 for b in data for i in range(8)]


def ble_crc_bits(pdu_bits: list[int]) -> list[int]:
    """CRC-24 as a 24-stage LFSR, positions 0..23, preset 0x555555 with
    position 0 holding the LSB of the preset; output position 23 first."""
    reg = [(0x555555 >> i) & 1 for i in range(24)]
    taps = (1, 3, 4, 6, 9, 10)  # x + x^3 + x^4 + x^6 + x^9 + x^10
    for b in pdu_bits:
        fb = reg[23] ^ b
        reg = [fb] + reg[:23]
        for t in taps:
            reg[t] ^= fb
    return reg[::-1]


def ble_whiten_bits(bits: list[int], channel: int) -> list[int]:
    """x^7 + x^4 + 1, position 0 = 1, positions 1..6 = channel bits 5..0."""
    reg = [1] + [(channel >> (5 - i)) & 1 for i in range(6)]
    out = []
    for b in bits:
        o = reg[6]
        out.append(b ^ o)
        reg = [o] + reg[:6]
        reg[4] ^= o
    return out


def ble_adv_bits(channel: int, adv_addr: bytes, data: bytes) -> list[int]:
    pdu = bytes([0x42, 6 + len(data)]) + adv_addr + data
    body = _lsb_bits(pdu)
    body += ble_crc_bits(body)
    aa = _lsb_bits(BLE_AA.to_bytes(4, 'little'))
    pre = [1 - aa[0] if i % 2 == 0 else aa[0] for i in range(8)][::-1]
    return pre + aa + ble_whiten_bits(body, channel)


def crc8(data: bytes) -> int:
    c = 0xFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def nrf24_bits(addr: bytes, payload: bytes, crc_len: int) -> list[int]:
    body = addr + payload
    if crc_len == 1:
        body += bytes([crc8(body)])
    elif crc_len == 2:
        body += crc16(body).to_bytes(2, 'big')
    first = addr[0] >> 7
    pre = [1 - first if i % 2 == 0 else first for i in range(8)][::-1]
    return pre + _msb_bits(body)


# ---- modulation / demodulation -------------------------------------------

def gfsk_modulate(bits, fs: float, bitrate: float, dev_hz: float, offset_hz: float = 0.0, bt: float = 0.5,
                  pad_bits: int = 20, noise: float = 0.0, seed: int = 1) -> np.ndarray:
    sps = fs / bitrate
    nrz = np.repeat(2 * np.array([0] * pad_bits + list(bits) + [0] * pad_bits) - 1.0, int(round(sps)))
    t = (np.arange(-2 * sps, 2 * sps + 1)) / sps
    sigma = np.sqrt(np.log(2)) / (2 * np.pi * bt)
    g = np.exp(-t ** 2 / (2 * sigma ** 2))
    f = np.convolve(nrz, g / g.sum(), 'same') * dev_hz + offset_hz
    x = np.exp(2j * np.pi * np.cumsum(f) / fs)
    x[: pad_bits * int(sps)] = 0
    x[-(pad_bits - 2) * int(sps):] = 0  # the transmitter holds the carrier 2 bits past the packet
    rng = np.random.default_rng(seed)
    return x + noise * (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x)))


def instantaneous_freq(iq: np.ndarray, fs: float, offset_hz: float = 0.0, bitrate: float | None = None) -> np.ndarray:
    x = iq * np.exp(-2j * np.pi * offset_hz * np.arange(len(iq)) / fs)
    if bitrate:  # channel filter: boxcar half a bit long (first null at 2 x bitrate)
        m = max(1, int(fs / bitrate / 2))
        x = np.convolve(x, np.ones(m) / m, 'same')
    return np.angle(x[1:] * np.conj(x[:-1])) * fs / (2 * np.pi)


def demod_bits(iq: np.ndarray, fs: float, bitrate: float, offset_hz: float = 0.0) -> list[list[int]]:
    """One bit list per sampling phase, widest eye opening first."""
    sps = fs / bitrate
    f = instantaneous_freq(iq, fs, offset_hz, bitrate)
    n = max(1, int(round(sps)))
    smooth = np.convolve(f, np.ones(n) / n, 'same')
    # Remove the carrier offset (crystal tolerance, drift): slice midway
    # between the two tones, measured where the signal is present.
    power = np.convolve(np.abs(iq[1:]) ** 2, np.ones(n) / n, 'same')
    on = power > 0.25 * power.max()
    if on.sum() > 4 * n:
        lo, hi = np.percentile(smooth[on], [10, 90])
        smooth = smooth - (lo + hi) / 2
    phases = []
    for ph in range(n):
        idx = (ph + np.arange(int((len(smooth) - ph) / sps)) * sps).astype(int)
        v = smooth[idx]
        phases.append((np.median(np.abs(v)), [int(x > 0) for x in v]))
    return [bits for _, bits in sorted(phases, key=lambda p: -p[0])]


def _find(bits: list[int], pattern: list[int]) -> list[int]:
    s = ''.join(map(str, bits))
    p = ''.join(map(str, pattern))
    out, i = [], s.find(p)
    while i >= 0:
        out.append(i)
        i = s.find(p, i + 1)
    return out


def _bytes_lsb(bits: list[int]) -> bytes:
    return bytes(sum(bits[8 * k + i] << i for i in range(8)) for k in range(len(bits) // 8))


def _bytes_msb(bits: list[int]) -> bytes:
    return bytes(sum(bits[8 * k + i] << (7 - i) for i in range(8)) for k in range(len(bits) // 8))


def _dedupe(found: dict) -> list[dict]:
    """Sampling phases report one packet a bit or two apart: keep, per cluster,
    the first one found (phases are searched widest eye first)."""
    rank = {pos: r for r, pos in enumerate(found)}
    clusters, last = [], None
    for pos in sorted(found):
        if last is None or pos - last > 8:
            clusters.append([])
        clusters[-1].append(pos)
        last = pos
    return [found[min(c, key=rank.get)] for c in clusters]


def ble_decode(phases: list[list[int]], channel: int) -> list[dict]:
    """Every packet with a valid CRC-24 found in any sampling phase (deduplicated by position)."""
    aa = _lsb_bits(BLE_AA.to_bytes(4, 'little'))
    found = {}
    for bits in phases:
        for i in _find(bits, aa):
            rest = bits[i + 32:]
            if len(rest) < 16:
                continue
            head = ble_whiten_bits(rest[:16], channel)
            length = _bytes_lsb(head)[1] & 0x3F
            total = 16 + 8 * length + 24
            if len(rest) < total:
                continue
            body = ble_whiten_bits(rest[:total], channel)
            ok = ble_crc_bits(body[:-24]) == body[-24:]
            if ok and i not in found:
                pdu = _bytes_lsb(body[:-24])
                found[i] = {'type': pdu[0] & 0x0F, 'adv_addr': pdu[2:8], 'data': pdu[8:], 'bit': i}
    return _dedupe(found)


def nrf24_decode(phases: list[list[int]], addr: bytes, payload_len: int, crc_len: int,
                 expect: bytes | None = None) -> list[dict]:
    """Every packet for `addr` whose CRC checks. Without a CRC every address
    match counts, unless `expect` (a known test payload) stands in for the CRC."""
    a = _msb_bits(addr)
    found = {}
    for bits in phases:
        for i in _find(bits, a):
            total = 8 * (len(addr) + payload_len + crc_len)
            frame = bits[i:i + total]
            if len(frame) < total:
                continue
            raw = _bytes_msb(frame)
            body, crc = raw[:len(raw) - crc_len], raw[len(raw) - crc_len:]
            ok = ((crc_len == 0 and (expect is None or body[len(addr):] == expect)) or (crc_len == 1 and crc[0] == crc8(body))
                  or (crc_len == 2 and int.from_bytes(crc, 'big') == crc16(body)))
            if ok and i not in found:  # earlier phases have the wider eye
                found[i] = {'payload': body[len(addr):], 'bit': i}
    return _dedupe(found)


def _self_test():
    fs = 16e6
    adv = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0xC6])
    data = bytes([2, 1, 6, 7, 9]) + b'ESPSDR'
    for ch in (37, 38, 39):
        iq = gfsk_modulate(ble_adv_bits(ch, adv, data), fs, 1e6, 250e3, offset_hz=2e6 + 40e3, noise=0.2)
        got = ble_decode(demod_bits(iq, fs, 1e6, 2e6), ch)
        assert len(got) == 1 and got[0]['adv_addr'] == adv and got[0]['data'] == data, (ch, got)
    addr = bytes([0xE7, 0xE7, 0xE7, 0xE7, 0xE7])
    for rate, dev in ((250e3, 160e3), (1e6, 160e3), (2e6, 320e3)):
        for crc_len in (0, 1, 2):
            pl = bytes(range(1, 9))
            iq = gfsk_modulate(nrf24_bits(addr, pl, crc_len), fs, rate, dev, offset_hz=-1.5e6 - 30e3, noise=0.2)
            got = nrf24_decode(demod_bits(iq, fs, rate, -1.5e6), addr, len(pl), crc_len)
            assert len(got) == 1 and got[0]['payload'] == pl, (rate, crc_len, got)
    # A corrupted CRC must not decode.
    bits = ble_adv_bits(37, adv, data)
    bits[-1] ^= 1
    assert not ble_decode(demod_bits(gfsk_modulate(bits, fs, 1e6, 250e3), fs, 1e6), 37)
    print('demod self-test OK')


if __name__ == '__main__':
    _self_test()
