#!/usr/bin/env python3
"""Record performance/behaviour of the legacy (pre-rewrite) firmware.

Usage: python3 tests/hw/baseline_legacy.py /dev/ttyACM0 tests/hw/baseline.json
Each metric is the median of REPEAT runs. The rewritten firmware is compared
against this file by tests/hw/parity.py.
"""
import json, statistics, sys, time, zlib
import numpy as np
import serial

PORT, OUT = sys.argv[1], sys.argv[2]
REPEAT = 3
RATES = {6: 16_000_000, 1: 40_000_000, 0: 80_000_000}
s = serial.Serial(PORT, 2000000, timeout=5)
time.sleep(2); s.reset_input_buffer()


def cmd(line):
    s.write((line + '\n').encode())
    return s.readline().decode(errors='replace').strip()


def run_report(line, end_tag, timeout=10):
    """Send a streaming command, swallow binary output, return the end report fields."""
    s.write((line + '\n').encode())
    s.timeout = timeout
    s.read_until(end_tag.encode() + b' ')
    fields = [int(v) for v in s.readline().split()]
    s.timeout = 5
    keys = ('status detail units pairs elapsed_us late_max work_max frames drops abandoned '
            'ffts stopped').split()
    return dict(zip(keys, fields))


def med(values):
    return statistics.median(values)


base = {}
cmd('FREQ 2437')
base['gain'] = cmd('GAIN?')

caps = {}
for rate in RATES:
    times, rms = [], []
    for _ in range(REPEAT):
        s.write(f'CAP 16380 {rate}\n'.encode())
        head = s.readline().split()
        data = s.read(16380 * 4)
        assert head[0] == b'DATA' and int(head[2], 16) == zlib.crc32(data), head
        words = np.frombuffer(data, '<u4')
        i = (words & 0x3ff).astype(int); q = ((words >> 10) & 0x3ff).astype(int)
        i[i >= 512] -= 1024; q[q >= 512] -= 1024
        x = i + 1j * q
        times.append(int(head[3])); rms.append(float(np.std(x)))
    caps[RATES[rate]] = {'elapsed_us': med(times), 'rms': med(rms)}
base['capture_16380'] = caps

rings = {}
for rate in RATES:
    runs = [run_report(f'RING 2000 {rate}', 'RING') for _ in range(REPEAT)]
    rings[RATES[rate]] = {k: med([r[k] for r in runs]) for k in ('status', 'pairs', 'late_max', 'elapsed_us')}
base['ring_2s'] = rings

ringcap = {}
for rate in RATES:
    ok = 0
    for _ in range(REPEAT):
        s.write(f'RINGCAP 3 {rate}\n'.encode())
        h = s.readline().split()
        n = sum(int(v) for v in h[3:6])
        data = s.read(n * 4)
        rep = s.readline().split()
        ok += int(h[0] == b'RINGDATA' and int(h[6], 16) == zlib.crc32(data) and rep[1] == b'0')
    ringcap[RATES[rate]] = ok
base['ringcap_ok_of_3'] = ringcap

specinfo = json.loads(cmd('SPECINFO?').split(' ', 1)[1])
spec = {}
for fs, rate, nfft, stride, upf in specinfo['profiles']:
    runs = [run_report(f'SPEC 1000 {stride} {upf} 0 {rate} {nfft}', 'SPECEND') for _ in range(REPEAT)]
    key = f'{fs // 1_000_000}M_{nfft}'
    spec[key] = {k: med([r[k] for r in runs]) for k in ('status', 'pairs', 'frames', 'drops', 'abandoned', 'ffts')}
    done = spec[key]['ffts']; spec[key]['processed_pct'] = round(100 * done / max(done + spec[key]['abandoned'], 1), 1)
base['spec_1s'] = spec

iqs = {}
for dec in (64, 256):
    runs = [run_report(f'IQS 1000 {dec} 8 6', 'IQSEND') for _ in range(REPEAT)]
    iqs[dec] = {k: med([r[k] for r in runs]) for k in ('status', 'pairs', 'frames', 'drops', 'abandoned', 'ffts')}
base['iqs_1s_16M_8bit'] = iqs  # 'ffts' field carries lost pairs in IQS mode

json.dump(base, open(OUT, 'w'), indent=1)
print(json.dumps(base, indent=1))
