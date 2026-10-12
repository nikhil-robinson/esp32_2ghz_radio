#!/usr/bin/env python3
"""Hardware parity suite: every capability of the rewritten firmware, checked
against tests/hw/baseline.json (measured on the legacy firmware).

Usage: python3 tests/hw/parity.py /dev/ttyACM0 [--quick]
Exit status 0 only if every check passes. Not part of the host unit tests.
"""
import json
import statistics
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'host'))
from esp_sdr import Device, DeviceError, decode_iq8, decode_iq10, decode_iq32  # noqa: E402
from esp_sdr import protocol as p  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
QUICK = '--quick' in sys.argv
REPEAT = 1 if QUICK else 3
BASE = json.loads((ROOT / 'tests/hw/baseline.json').read_text())
WIRE_RATE = {80_000_000: 0, 40_000_000: 1, 16_000_000: 2}
LATE_LIMIT = 2000
failures = []


def check(name, ok, detail=''):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        failures.append(name)


def expect_error(dev, name, error, **fields):
    try:
        dev.request(name, **fields)
    except DeviceError as e:
        return f'error {error}' in str(e)
    return False


def expect_error_run(dev, name, **fields):
    try:
        run(dev, name, **fields)
    except DeviceError:
        return True
    return False


def run(dev, name, **fields):
    frames = list(dev.stream(name, **fields))
    return frames, dev.last_run


def median_run(dev, name, keys, **fields):
    runs = [run(dev, name, **fields)[1] for _ in range(REPEAT)]
    return {k: statistics.median(r[k] for r in runs) for k in keys}, runs


with Device(PORT) as dev:
    time.sleep(0.5)
    dev.serial.reset_input_buffer()

    # ---- identity, settings, errors -----------------------------------------
    info = dev.info()
    check('INFO', info['protocol'] == p.PROTOCOL_VERSION and info['min_mhz'] == 1840 and info['max_mhz'] == 2790,
          f"gain_max={info['gain_max']} {info['version'][:40]}")
    for mhz in (1840, 2209, 2210, 2437, 2440, 2790):  # direct, 6/5 divider edge, channel, non-channel
        dev.set_frequency(mhz)
        iq, _ = dev.capture(4096, 2)
        check(f'tune + capture {mhz} MHz', 1 < np.std(iq) < 400, f'std={np.std(iq):.1f}')
    check('reject 1839 MHz', expect_error(dev, 'SET_FREQUENCY', 3, mhz=1839))
    check('reject 2791 MHz', expect_error(dev, 'SET_FREQUENCY', 3, mhz=2791))
    dev.set_frequency(2437)

    g = dev.gain()
    dev.set_gain(30)
    g2 = dev.gain()
    dev.set_gain(None)
    g3 = dev.gain()
    check('gain manual 30 / back to AGC', g2['manual'] == 1 and g2['index'] == 30 and g2['forced'] == 1 and
          g3['manual'] == 0, f"max={g['max_index']}")
    check('reject gain above max', expect_error(dev, 'SET_GAIN', 3, manual=1, index=g['max_index'] + 1))

    dev.request('SET_BANDWIDTH', mhz=21)
    f = dev.request('GET_FILTER', 'FILTER')
    check('bandwidth 21 MHz -> filter code 32', f['code'] == 32, str({k: f[k] for k in ('code', 'active_i')}))
    dev.request('SET_FILTER', code=-1)
    check('filter back to automatic', dev.request('GET_FILTER', 'FILTER')['code'] == -1)
    check('reject bandwidth 5 MHz', expect_error(dev, 'SET_BANDWIDTH', 3, mhz=5))

    w = dev.request('GET_WORKERS', 'WORKERS')
    check('core 1 alive, dual+assist default', w['core1_alive'] == 1 and w['mode'] == 1)
    mem = dev.request('GET_MEMORY', 'MEMORY')
    check('memory report', mem['banks'] == 3 and mem['threshold'] == 12288 and mem['heap_free'] > 20000,
          f"heap_free={mem['heap_free']}")

    pins = dev.request('GET_GPIO', 'GPIO')['_body']
    free = pins[0::2]
    check('GPIO list excludes memory/USB pins', len(free) > 5 and not set(free) & {19, 20, *range(26, 38)},
          f'{len(free)} free pins')
    pin = free[0]
    dev.request('SET_GPIO', pin=pin, state=2)
    state = dict(zip(*[iter(dev.request('GET_GPIO', 'GPIO')['_body'])] * 2))[pin]
    dev.request('SET_GPIO', pin=pin, state=0)
    check(f'GPIO {pin} high then float', state == 2)
    check('reject GPIO 19 (USB)', expect_error(dev, 'SET_GPIO', 3, pin=19, state=1))

    # malformed requests: unknown type, wrong payload length, bad CRC, oversize
    seq = dev.send('GET_INFO')
    dev.serial.write(struct.pack('<HBBI', p.MAGIC, 0x7E, 9, 0) + struct.pack('<I', zlib.crc32(bytes([0x7E, 9, 0, 0, 0, 0]))))
    frames = [dev.read_frame(), dev.read_frame()]
    check('unknown request -> ERR_UNKNOWN_COMMAND', any(fr.seq == 9 and fr.fields()['error'] == 1 for fr in frames))
    body = struct.pack('<HBBI', p.MAGIC, 0x10, 10, 1) + b'\x01'
    dev.serial.write(body + struct.pack('<I', zlib.crc32(body[2:])))
    check('wrong payload length -> ERR_BAD_LENGTH', dev.read_frame().fields()['error'] == 2)
    dev.serial.write(struct.pack('<HBBI', p.MAGIC, 0x01, 11, 0) + b'\0\0\0\0')
    dev.serial.write(b'\x00garbage\xe5')
    check('bad CRC ignored, parser resyncs', dev.info()['protocol'] == p.PROTOCOL_VERSION)
    dev.serial.write(struct.pack('<HBBI', p.MAGIC, 0x01, 12, 100000))
    check('oversized request -> ERR_BAD_LENGTH', dev.read_frame().fields()['error'] == 2)
    check('STOP while idle -> ACK', dev.request('STOP')['error'] == 0)

    # ---- burst capture --------------------------------------------------------
    for hz, rate in WIRE_RATE.items():
        times = []
        for _ in range(REPEAT):
            (meta, data), = dev.capture_raw(16380, rate)
            times.append(meta['elapsed_us'])
        base_us = BASE['capture_16380'][str(hz)]['elapsed_us']
        t = statistics.median(times)
        iq = decode_iq32(np.frombuffer(data, '<u4'))
        check(f'capture 16380 @ {hz // 10**6} Msps time <= baseline', t <= base_us * 1.02 + 2,
              f'{t} us vs {base_us} us, std={np.std(iq):.1f}')
    raw = dev.capture_raw(4096, 2, fmt=0, repeats=3)
    check('capture repeats=3 -> 3 DATA frames', len(raw) == 3)
    (m10, d10), = dev.capture_raw(4097, 2, fmt=1)
    (m8, d8), = dev.capture_raw(4096, 2, fmt=2)
    w10 = decode_iq10(d10, 4097)
    check('IQ10 packed: size and plausible samples', len(d10) == (4097 * 20 + 7) // 8 and
          1 < np.std(decode_iq32(w10)) < 400)
    check('IQ8: size and plausible samples', len(d8) == 8192 and 0.2 < np.std(decode_iq8(d8)) < 128)
    dev.send('CAPTURE', rate=2, format=0, samples=255, repeats=1)
    check('reject 255 samples (RUN_DONE error)', dev.read_frame().fields()['error'] == 3)

    # ---- continuous ring ------------------------------------------------------
    for hz, rate in WIRE_RATE.items():
        r, _ = median_run(dev, 'RING_STATS', ('status', 'pairs', 'late_max'), rate=rate, duration_ms=2000)
        b = BASE['ring_2s'][str(hz)]
        check(f'ring 2 s @ {hz // 10**6} Msps gapless', r['status'] == 0 and r['pairs'] >= b['pairs'] * 0.98 and
              r['late_max'] <= LATE_LIMIT, f"pairs={r['pairs']} (base {b['pairs']}) late_max={r['late_max']}")
    for hz, rate in WIRE_RATE.items():
        ok = 0
        for _ in range(REPEAT):
            frames, done = run(dev, 'RING_CAPTURE', rate=rate, units=3)
            data = [fr for fr in frames if fr.message.name == 'RING_DATA']
            if done['status'] == 0 and len(data) == 1:
                v = data[0].fields()
                ok += v['units'] == 3 and len(data[0].body()) == 4 * (v['count0'] + v['count1'] + v['count2'])
        check(f'ring capture 3 units @ {hz // 10**6} Msps', ok == REPEAT, f'{ok}/{REPEAT}')

    # ---- spectrum -------------------------------------------------------------
    for key, b in BASE['spectrum_1s' if 'spectrum_1s' in BASE else 'spec_1s'].items():
        msps, nfft = key.split('_')
        rate = WIRE_RATE[int(msps[:-1]) * 10**6]
        r, runs = median_run(dev, 'SPECTRUM', ('status', 'frames', 'drops', 'abandoned', 'ffts'),
                             rate=rate, nfft=int(nfft), duration_ms=1000)
        pct = 100 * r['ffts'] / max(r['ffts'] + r['abandoned'], 1)
        # Frames are emitted whenever the output queue drains, so a faster host gets more
        # frames and core 1 spends that time encoding them instead of on FFTs. The legacy
        # baseline used a slow byte-wise reader; compare coverage at >= its frame rate.
        check(f'spectrum {key}: coverage within 3 pts at >= baseline frame rate, no drops',
              r['status'] == 0 and r['drops'] == 0 and r['frames'] >= b['frames'] and
              pct >= b['processed_pct'] - 3,
              f"{pct:.1f}% (base {b['processed_pct']}%) frames={r['frames']} (base {b['frames']})")
    frames, done = run(dev, 'SPECTRUM', rate=1, nfft=1024, duration_ms=1500, stats=1, max_hold=1)
    kinds = [fr.message.name for fr in frames]
    spec = [fr for fr in frames if fr.message.name == 'SPECTRUM_FRAME']
    check('spectrum frames: bins + max-hold flag + STATS', spec and len(spec[0].body()) == 1024 and
          spec[0].fields()['flags'] & 1 and 'STATS' in kinds, f"{kinds.count('STATS')} stats frames")
    for mode in (0, 2):
        dev.request('SET_WORKERS', mode=mode)
        _, d = run(dev, 'SPECTRUM', rate=2, nfft=512, duration_ms=500)
        check(f'spectrum with workers={mode}', d['status'] == 0 and d['ffts'] > 0, f"ffts={d['ffts']}")
    dev.request('SET_WORKERS', mode=1)
    # open-ended run stopped by the host
    seq = dev.send('SPECTRUM', rate=2, nfft=256, duration_ms=0)
    time.sleep(0.4)
    dev.stop()
    while True:
        fr = dev.read_frame()
        if fr.seq == seq and fr.message.name == 'RUN_DONE':
            break
    check('open-ended spectrum stops on host request', fr.fields()['stopped_by_host'] == 1)
    check('device responsive after stop', dev.info()['protocol'] == p.PROTOCOL_VERSION)

    # ---- IQ stream -------------------------------------------------------------
    for dec in (64, 256):
        frames, d = run(dev, 'IQ_STREAM', rate=2, decimation=dec, bits=8, shift=255, duration_ms=1000)
        b = BASE['iqs_1s_16M_8bit'][str(dec)]
        iqf = [fr for fr in frames if fr.message.name == 'IQ_FRAME']
        samples = sum(fr.fields()['samples'] for fr in iqf)
        check(f'IQ stream dec {dec}: delivered >= baseline', d['status'] == 0 and d['frames'] >= b['frames'] * 0.9,
              f"frames={d['frames']} drops={d['drops']} (base {b['frames']}/{b['drops']}) samples={samples}")
    _, d = run(dev, 'IQ_STREAM', rate=2, decimation=100, bits=8, shift=255, duration_ms=200)
    check('IQ stream rejects decimation 100', d['status'] == p.RUN_STATUS['RUN_FAIL_ARG'])

    # ---- full-band scan --------------------------------------------------------
    for rate, start, budget_s in ((1, 1842, 3.0), (0, 1844, 2.0)):
        t = time.time()
        f, db, r = dev.scan(start, 2790, rate=rate)
        dt = time.time() - t
        steps = np.diff(f)
        check(f'scan {start}-2790 MHz rate {rate}: contiguous, in time', r['error'] == 0 and f[0] >= start * 1e6 and
              f[-1] <= 2790e6 and steps.max() < 2 * np.median(steps) and dt < budget_s,
              f'{len(f)} bins, {dt:.1f} s')
    check('scan rejects LO below range', expect_error_run(dev, 'SCAN', from_mhz=1840, to_mhz=1900, rate=1, nfft=1024,
                                                         averages=4, gain=40, sweeps=1))

print(f"\n{'ALL PASS' if not failures else f'{len(failures)} FAILED: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
