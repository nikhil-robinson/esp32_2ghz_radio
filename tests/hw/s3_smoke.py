#!/usr/bin/env python3
"""On-hardware smoke test for the ESP32-S3 receiver over USB-Serial-JTAG.

Usage: python3 tests/hw/s3_smoke.py [/dev/ttyACM0]
Exits non-zero on the first failed check. Not part of `unittest discover`.
"""
import sys, time, zlib
import numpy as np
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
s = serial.Serial(PORT, 2000000, timeout=3)
failures = 0


def check(name, ok, detail=''):
    global failures
    failures += not ok
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")


def cmd(line):
    s.write((line + '\n').encode())
    return s.readline().decode(errors='replace').strip()


def read_exact(n):
    buf = s.read(n)
    if len(buf) != n:
        raise TimeoutError(f'wanted {n} bytes, got {len(buf)}')
    return buf


def iq_from_words(words):
    """Raw word: I in bits 0-9, Q in bits 10-19, 10-bit two's complement."""
    i = (words & 0x3ff).astype(np.int32); q = ((words >> 10) & 0x3ff).astype(np.int32)
    i[i >= 512] -= 1024; q[q >= 512] -= 1024
    return i + 1j * q


def unpack_iq10(buf, n):
    """Inverse of pack_iq() in receiver.c: two 20-bit words per five bytes."""
    b = np.frombuffer(buf + b'\0' * 2, np.uint8).astype(np.uint32)
    words = np.empty(n, np.uint32)
    for j in range(0, n, 2):
        p = b[j // 2 * 5:]
        words[j] = p[0] | p[1] << 8 | (p[2] & 15) << 16
        if j + 1 < n:
            words[j + 1] = p[2] >> 4 | p[3] << 4 | p[4] << 12
    return words


def capture(n, rate, kind='CAP'):
    s.write(f'{kind} {n} {rate}\n'.encode())
    head = s.readline().decode().split()
    if not head or head[0] != 'DATA':
        raise RuntimeError(f'{kind}: {head}')
    nbytes = n * 4 if kind == 'CAP' else (n * 20 + 7) // 8
    data = read_exact(nbytes)
    return int(head[2], 16), zlib.crc32(data), data, int(head[3])


time.sleep(2); s.reset_input_buffer()  # drain boot log
s.write(b'\n'); time.sleep(0.2); s.reset_input_buffer()

# 1. Command layer
r = cmd('INFO'); check('INFO', r.startswith('S3SDR'), r)
r = cmd('CAPS'); check('CAPS lists SPEC/RING', 'SPEC' in r and 'RING' in r, r[:60] + '...')
r = cmd('RANGE?'); check('RANGE?', r.startswith('RANGE'), r)
r = cmd('VERSION?'); check('VERSION?', r.startswith('VERSION'), r[:80])
r = cmd('TRANSPORT?'); check('TRANSPORT is USB', r.startswith('TRANSPORT USB'), r)
r = cmd('BOGUS'); check('unknown command rejected', r == 'ERR command', r)
r = cmd('FREQ 2437'); check('tune 2437 MHz (Wi-Fi ch6)', r == 'OK', r)

# 2. Raw burst capture, all three native rates
for rate, msps in ((6, 16), (1, 40), (0, 80)):
    crc_dev, crc_host, data, us = capture(4096, rate)
    iq = iq_from_words(np.frombuffer(data, '<u4'))
    sentinel = np.count_nonzero(np.frombuffer(data, '<u4') == 0xa5a0055a)
    rms = np.sqrt(np.mean(np.abs(iq - iq.mean()) ** 2))
    check(f'CAP 4096 @ {msps} Msps CRC', crc_dev == crc_host, f'{crc_dev:08x}')
    check(f'CAP @ {msps} Msps live samples', sentinel == 0 and 1 < rms < 400,
          f'rms={rms:.1f} dc=({iq.real.mean():.1f},{iq.imag.mean():.1f}) {us} us')

# 3. Packed IQ10 path matches raw-word layout
crc_dev, crc_host, data, _ = capture(4096, 6, 'CAP20')
check('CAP20 packed CRC', crc_dev == crc_host)
iq = iq_from_words(unpack_iq10(data, 4096))
check('CAP20 decodes to plausible IQ', 1 < np.std(iq) < 400, f'std={np.std(iq):.1f}')

# 4. Second core (exercises the IDF 5.3 APP-CPU bring-up shim)
r = cmd('DUAL?'); check('core 1 alive', r.split()[-1] == '1', r)

# 5. Continuous ring: 500 ms at 16 Msps must not drop
r = cmd('RING 500 6').split()
check('RING 500 ms status 0, 0 drops', r[0] == 'RING' and r[1] == '0' and r[9] == '0' and r[10] == '0', ' '.join(r))

# 6. Gapless ring capture with CRC
s.write(b'RINGCAP 2 6\n')
h = s.readline().decode().split()
if h and h[0] == 'RINGDATA':
    pairs = sum(map(int, h[3:6])); data = read_exact(pairs * 4)
    check('RINGCAP CRC', int(h[6], 16) == zlib.crc32(data), f'{pairs} words')
    rep = s.readline().decode().split(); check('RINGCAP status 0', rep[1:2] == ['0'], ' '.join(rep))
else:
    check('RINGCAP header', False, str(h))

# 7. On-chip FFT spectrum: 1 s, 1024 bins, 40 Msps, SPECINFO's dual-core profile
s.write(b'SPEC 1000 5 9 0 1 1024\n')
hdr = s.readline().decode().strip(); check('SPEC header', hdr.startswith('SPEC 1024'), hdr)
s.timeout = 5
buf = s.read_until(b'SPECEND ')
tail = s.readline().decode().split()
s.timeout = 3
check('SPEC run status 0', tail[:1] == ['0'], 'SPECEND ' + ' '.join(tail))
if len(tail) > 9:
    # status detail units pairs elapsed late_max work_max frames drops abandoned ffts stopped
    # Abandoned work is the designed back-pressure path, so report it, don't fail on it.
    ffts, ab = int(tail[10]), int(tail[9])
    check('SPEC produced FFTs, 0 frame drops', ffts > 0 and tail[8] == '0',
          f'pairs={tail[3]} frames={tail[7]} drops={tail[8]} ffts={ffts} abandoned={ab} '
          f'({100 * ffts / max(ffts + ab, 1):.0f}% of blocks processed)')

# 8. RF sanity: the 2.4 GHz band should not look identical everywhere
powers = {}
for f in (2412, 2437, 2462, 2480):
    cmd(f'FREQ {f}')
    _, _, data, _ = capture(16000, 1)
    iq = iq_from_words(np.frombuffer(data, '<u4'))
    powers[f] = 10 * np.log10(np.mean(np.abs(iq - iq.mean()) ** 2) + 1e-9)
print('      band power (dB rel. ADC):', {k: round(v, 1) for k, v in powers.items()})
check('tuning changes what we hear', np.ptp(list(powers.values())) > 0.3)

print(f'\n{"ALL PASS" if not failures else f"{failures} FAILED"}')
sys.exit(1 if failures else 0)
