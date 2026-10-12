"""Protocol single source of truth: generated headers and the Python client."""
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'host'))
from esp_sdr import device as dev_mod  # noqa: E402
from esp_sdr import protocol as p  # noqa: E402


class GeneratedHeaders(unittest.TestCase):
    def test_headers_up_to_date(self):
        result = subprocess.run([sys.executable, str(ROOT / 'tools/gen_protocol.py'), '--check'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(shutil.which('gcc'), 'gcc not available')
    def test_struct_sizes_match_python(self):
        """The C static asserts encode Python's struct sizes; compiling proves they agree."""
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'sizes.c'
            src.write_text('#include "protocol.h"\nint main(void) { return 0; }\n')
            subprocess.run(['gcc', '-std=c17', '-Wall', '-Werror', f'-I{ROOT / "components/esp_sdr/include"}',
                            f'-I{ROOT / "firmware/main"}', str(src), '-o', str(Path(tmp) / 'sizes')], check=True)

    def test_ids_unique(self):
        ids = [m.id for m in p.ALL]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertTrue(all(m.size <= p.MAX_REQUEST_PAYLOAD for m in p.REQUESTS))


def frame(type_, seq, payload=b''):
    head = struct.pack('<HBBI', p.MAGIC, type_, seq, len(payload))
    return head + payload + struct.pack('<I', zlib.crc32(head[2:] + payload))


class FakeSerial:
    def __init__(self, data: bytes):
        self.data = bytearray(data)
        self.written = b''

    def read(self, n=1):
        out, self.data = bytes(self.data[:n]), self.data[n:]
        return out

    def write(self, b):
        self.written += b


class Client(unittest.TestCase):
    def make(self, data):
        d = dev_mod.Device.__new__(dev_mod.Device)
        d.serial, d.timeout, d._seq = FakeSerial(data), 0.5, 0
        return d

    def test_request_skips_noise_and_other_seq(self):
        gain = p.BY_NAME['GAIN'].pack(manual=1, index=30, max_index=82, forced=1)
        d = self.make(b'boot log\r\n\xe5' + frame(0x82, 99, gain) + frame(0x82, 1, gain))
        self.assertEqual(d.request('GET_GAIN', 'GAIN')['index'], 30)
        sent = d.serial.written
        self.assertEqual(sent[:2], b'\xe5\xd5')
        self.assertEqual(zlib.crc32(sent[2:-4]), struct.unpack('<I', sent[-4:])[0])

    def test_ack_error_raises(self):
        ack = p.BY_NAME['ACK'].pack(error=3, reserved=0, request=0x10, reserved2=0)
        d = self.make(frame(0x80, 1, ack))
        with self.assertRaises(dev_mod.DeviceError):
            d.set_frequency(9999)

    def test_crc_mismatch_detected(self):
        bad = bytearray(frame(0x82, 1, b'\0' * 4))
        bad[9] ^= 0xff
        d = self.make(bytes(bad))
        with self.assertRaises(dev_mod.DeviceError):
            d.request('GET_GAIN', 'GAIN')

    def test_stream_collects_until_run_done(self):
        rec = p.BY_NAME['SPECTRUM_FRAME']
        spec = rec.pack(frame=0, pair_index=0, pairs=12288, ffts=4, flags=0, gain=0, drops=0, nfft_log2=8,
                        db_step=2) + bytes(256)
        done_fields = {n: 0 for n, _ in p.BY_NAME['RUN_DONE'].fields}
        done = p.BY_NAME['RUN_DONE'].pack(**{**done_fields, 'ffts': 4})
        d = self.make(frame(0xA2, 1, spec) + frame(0xA2, 1, spec) + frame(0xA5, 1, done))
        frames = list(d.stream('SPECTRUM', rate=2, nfft=256, duration_ms=10))
        self.assertEqual(len(frames), 2)
        self.assertEqual(len(frames[0].body()), 256)
        self.assertEqual(d.last_run['ffts'], 4)

    def test_iq_decoders(self):
        import numpy as np
        words = np.array([(i & 0x3ff) | ((-i & 0x3ff) << 10) for i in range(-5, 6)], np.uint32)
        iq = dev_mod.decode_iq32(words)
        self.assertEqual(iq[0], -5 + 5j)
        packed = bytearray()
        for j in range(0, len(words), 2):
            a = int(words[j]); b = int(words[j + 1]) if j + 1 < len(words) else 0
            packed += bytes([a & 255, a >> 8 & 255, (a >> 16) | (b << 4) & 255, b >> 4 & 255, b >> 12 & 255])
        self.assertTrue((dev_mod.decode_iq10(bytes(packed), len(words)) == words).all())


if __name__ == '__main__':
    unittest.main()
