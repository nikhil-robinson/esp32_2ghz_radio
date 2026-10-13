#!/usr/bin/env python3
"""Two-way link over the air, two boards in TXRX mode: one responds, the other
initiates exchanges and must get each reply (valid CRC) back.

Reports the reply rate and the turnaround both sides measured (responder:
their last bit -> our first; initiator: our last bit -> first bit of the reply).

Usage: python3 tests/hw/txrx_link.py <port A> <port B>   (each board takes both roles)
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'host'))
from esp_sdr import Device  # noqa: E402

CHANNEL_KHZ = 2_440_000
ADDR = bytes([0xE7, 0xD3, 0xF0, 0x35, 0x77])
TURNAROUND_US = 130
EXCHANGES = 200
MIN_RATE = 0.95
# 2 Mbps gives the 4 Msps detector only 2 samples per bit: the matching phase is
# sometimes at the eye edge and ~5-10 % of bodies fail CRC (known limitation).
MIN_RATE_2M = 0.85
RX_GAIN = 40
failures = []


def run(resp: Device, init: Device, label: str):
    # Fixed gain: AGC raises the wideband noise until the on-chip detector's
    # short filter cannot separate the bits (measured).
    resp.set_gain(RX_GAIN)
    init.set_gain(RX_GAIN)
    for bitrate in (250_000, 1_000_000, 2_000_000):
        for crc_len in (1, 2):
            reply = b'ACK' + bytes([bitrate // 250_000])
            seq = resp.link_respond(CHANNEL_KHZ, ADDR, reply, 8, bitrate, crc_len, TURNAROUND_US,
                                    duration_ms=60_000, count=EXCHANGES, wait=False)
            time.sleep(0.2)
            got, delays = 0, []
            for k in range(EXCHANGES):
                payload = bytes([k & 0xFF]) + b'PING' + bytes(3)
                rx, v = init.link_exchange(CHANNEL_KHZ, ADDR, payload, len(reply), bitrate, crc_len, timeout_us=2000)
                if rx == reply:
                    got += 1
                    delays.append(v['delay_ns'] / 1000)
            resp.stop()
            s = resp.wait_link_summary(seq, 10)
            ok = got >= (MIN_RATE_2M if bitrate == 2_000_000 else MIN_RATE) * EXCHANGES
            print(f"{'PASS' if ok else 'FAIL'}  {label} {bitrate // 1000:4d} kbps crc{crc_len}: {got}/{EXCHANGES} replies; "
                  f"responder served {s['served']} (turnaround {s['delay_min_ns'] / 1000:.1f}-"
                  f"{s['delay_max_ns'] / 1000:.1f} us, crc rejects {s['crc_failures']}); "
                  f"initiator saw reply after {min(delays, default=0):.1f}-{max(delays, default=0):.1f} us")
            if not ok:
                failures.append((label, bitrate, crc_len))


def main(port_a: str, port_b: str):
    a, b = Device(port_a), Device(port_b)
    a.set_mode(2)
    b.set_mode(2)
    run(a, b, 'A responds')
    run(b, a, 'B responds')
    a.set_mode(0)
    b.set_mode(0)
    print('ALL PASS' if not failures else f'{len(failures)} FAILED')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:3]))
