"""Host client for the ESP-SDR firmware (serial transport).

    from esp_sdr import Device
    with Device('/dev/ttyACM0') as sdr:
        sdr.set_frequency(2437)
        iq, info = sdr.capture(4096, rate=2)          # complex64 samples
        for kind, frame in sdr.spectrum(1024, duration_ms=1000):
            ...
"""
from __future__ import annotations

import struct
import time
import zlib
from dataclasses import dataclass
from typing import Iterator

import numpy as np
import serial

from . import protocol as p

_HEADER = struct.Struct('<HBBI')


class DeviceError(RuntimeError):
    pass


@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes

    @property
    def message(self) -> p.Message:
        return p.BY_ID[self.type]

    def fields(self) -> dict:
        return self.message.unpack(self.payload)

    def body(self) -> bytes:
        """Variable-length data after the fixed fields."""
        return self.payload[self.message.size:]


def decode_iq32(words: np.ndarray) -> np.ndarray:
    """Raw dump words -> complex samples. The hardware's raw Q is inverted
    (mirrored spectrum); the returned samples are corrected."""
    i = (words & 0x3ff).astype(np.int16)
    q = ((words >> 10) & 0x3ff).astype(np.int16)
    i[i >= 512] -= 1024
    q[q >= 512] -= 1024
    return (i - 1j * q).astype(np.complex64)


def decode_iq10(data: bytes, samples: int) -> np.ndarray:
    """Packed IQ10 (two 20-bit samples per 5 bytes) -> raw 20-bit words."""
    b = np.frombuffer(data + b'\0' * 5, np.uint8).astype(np.uint32)
    pairs = (samples + 1) // 2
    g = b[:pairs * 5].reshape(pairs, 5)
    a = g[:, 0] | g[:, 1] << 8 | (g[:, 2] & 15) << 16
    c = g[:, 2] >> 4 | g[:, 3] << 4 | g[:, 4] << 12
    return np.stack([a, c], 1).reshape(-1)[:samples]


def decode_iq8(data: bytes) -> np.ndarray:
    """CAPTURE format IQ8 (raw bytes, inverted Q) -> corrected samples."""
    v = np.frombuffer(data, np.int8).astype(np.float32)
    return (v[0::2] - 1j * v[1::2]).astype(np.complex64)


class Device:
    def __init__(self, port: str, baud: int = 2_000_000, timeout: float = 5.0):
        self.serial = serial.Serial(port, baud, timeout=timeout)
        self.timeout = timeout
        self._seq = 0
        self.resync()

    def resync(self):
        """End any stream a previous client left running and flush its output.

        A running stream treats incoming bytes as a stop request and discards
        them, so a request sent into it would never be answered."""
        self.send('STOP')
        time.sleep(0.7)  # stream drain deadline on the device is 0.5 s
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- framing ---------------------------------------------------------

    def send(self, name: str, **fields) -> int:
        m = p.BY_NAME[name]
        self._seq = (self._seq + 1) & 0xff
        payload = m.pack(**{n: fields.get(n, 0) for n, _ in m.fields})
        head = _HEADER.pack(p.MAGIC, m.id, self._seq, len(payload))
        crc = zlib.crc32(head[2:] + payload)
        self.serial.write(head + payload + struct.pack('<I', crc))
        return self._seq

    def read_frame(self, timeout: float | None = None) -> Frame:
        """Next valid frame; bytes before a magic (boot noise) are skipped."""
        deadline = time.monotonic() + (timeout or self.timeout)
        window = b''
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError('no frame from device')
            byte = self.serial.read(1)
            if not byte:
                continue
            window = (window + byte)[-2:]
            if window != b'\xe5\xd5':
                continue
            rest = self._read_exact(6, deadline)
            _, type_, seq, length = _HEADER.unpack(window + rest)
            if length > 64 * 1024 * 1024:
                window = b''
                continue
            payload = self._read_exact(length, deadline)
            crc = struct.unpack('<I', self._read_exact(4, deadline))[0]
            if zlib.crc32(rest[:6] + payload) != crc:
                raise DeviceError(f'CRC mismatch in frame type 0x{type_:02x}')
            return Frame(type_, seq, payload)

    def _read_exact(self, n: int, deadline: float) -> bytes:
        out = bytearray()
        while len(out) < n:
            if time.monotonic() > deadline:
                raise TimeoutError(f'wanted {n} bytes, got {len(out)}')
            out += self.serial.read(n - len(out))
        return bytes(out)

    def request(self, name: str, expect: str = 'ACK', **fields) -> dict:
        seq = self.send(name, **fields)
        while True:
            f = self.read_frame()
            if f.seq != seq:
                continue
            if f.type == p.BY_NAME['ACK'].id and expect != 'ACK':
                raise DeviceError(f'{name}: error {f.fields()["error"]}')
            if f.message.name != expect:
                raise DeviceError(f'{name}: expected {expect}, got {f.message.name}')
            values = f.fields()
            if expect == 'ACK' and values['error']:
                raise DeviceError(f'{name}: error {values["error"]}')
            values['_body'] = f.body()
            return values

    # ---- receiver --------------------------------------------------------

    def info(self) -> dict:
        v = self.request('GET_INFO', 'INFO')
        v['version'] = v.pop('_body').decode()
        return v

    def set_frequency(self, mhz: int):
        self.request('SET_FREQUENCY', mhz=mhz)

    def set_gain(self, index: int | None):
        """None = hardware AGC."""
        self.request('SET_GAIN', manual=int(index is not None), index=index or 0)

    def gain(self) -> dict:
        return self.request('GET_GAIN', 'GAIN')

    # ---- capture and streams ----------------------------------------------

    def capture_raw(self, samples: int, rate: int = 2, fmt: int = 0, repeats: int = 1) -> list[tuple[dict, bytes]]:
        seq = self.send('CAPTURE', rate=rate, format=fmt, samples=samples, repeats=repeats)
        out = []
        for f in self._run_frames(seq):
            if f.message.name == 'DATA':
                out.append((f.fields(), f.body()))
        self._check_done()
        return out

    def capture(self, samples: int, rate: int = 2) -> tuple[np.ndarray, dict]:
        (info, data), = self.capture_raw(samples, rate)
        return decode_iq32(np.frombuffer(data, '<u4')), info

    def stream(self, name: str, timeout: float = 30.0, **fields) -> Iterator[Frame]:
        """Frames of a stream request; ends after RUN_DONE (see last_run)."""
        seq = self.send(name, **fields)
        yield from self._run_frames(seq, timeout)
        self._check_done()

    def stop(self):
        """Ask a running stream to end (any byte does)."""
        self.send('STOP')

    def _run_frames(self, seq: int, timeout: float = 30.0) -> Iterator[Frame]:
        self.last_run = None
        while True:
            f = self.read_frame(timeout)
            if f.seq != seq:
                continue
            if f.message.name == 'RUN_DONE':
                self.last_run = f.fields()
                return
            if f.message.name == 'ACK':
                raise DeviceError(f'request rejected: error {f.fields()["error"]}')
            yield f

    def _check_done(self):
        r = self.last_run
        if r is None:
            raise DeviceError('stream ended without RUN_DONE')
        if r['error']:
            raise DeviceError(f'request failed: error {r["error"]}')
