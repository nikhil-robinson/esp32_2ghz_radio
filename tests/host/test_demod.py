"""Self-test of the independent packet decoder used by tests/hw/tx_rf.py."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import demod  # noqa: E402


class DemodSelfTest(unittest.TestCase):
    def test_synthetic_packets(self):
        demod._self_test()


if __name__ == '__main__':
    unittest.main()
