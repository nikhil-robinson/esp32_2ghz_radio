"""Builds and runs tests/host/unit_tests.c with the host compiler."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / 'components/esp_sdr'
SOURCES = [ROOT / 'tests/host/unit_tests.c', ROOT / 'firmware/main/frame.c',
           *(LIB / 'src/dsp' / f for f in ('bandwidth.c', 'db_code.c', 'fir_design.c', 'iq_pack.c'))]
INCLUDES = [LIB / 'include', LIB / 'src/dsp', ROOT / 'firmware/main']


@unittest.skipUnless(shutil.which('gcc'), 'gcc not available')
class CUnitTests(unittest.TestCase):
    def test_pure_modules(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / 'unit_tests'
            subprocess.run(['gcc', '-std=c17', '-O1', '-Wall', '-Wextra', '-Werror', *(f'-I{i}' for i in INCLUDES),
                            *map(str, SOURCES), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
