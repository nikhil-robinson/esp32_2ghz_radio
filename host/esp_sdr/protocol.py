"""ESP-SDR data records and firmware wire protocol: the single source of truth.

`tools/gen_protocol.py` generates two C headers from these tables:
  components/esp_sdr/include/esp_sdr_record.h  - stream records (library data format)
  firmware/main/protocol.h                       - firmware requests / responses
A host test fails if either checked-in header differs from the generated one.

Stream records and firmware messages share one frame layout, so a host reads
both with the same parser.

Frame (both directions, little endian):

    magic u16 (0xD5E5 -> bytes E5 D5) | type u8 | seq u8 | length u32 | payload | crc32 u32

crc32 is zlib/IEEE CRC-32 over type, seq, length and payload.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = 0xD5E5
HEADER_SIZE = 8
CRC_SIZE = 4
MAX_REQUEST_PAYLOAD = 64
PROTOCOL_VERSION = 1

# C type -> struct format character.
_CTYPES = {'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h',
           'uint32_t': 'I', 'int32_t': 'i', 'uint64_t': 'Q'}


@dataclass(frozen=True)
class Message:
    name: str
    id: int
    fields: tuple[tuple[str, str], ...] = ()
    doc: str = ''

    @property
    def format(self) -> str:
        return '<' + ''.join(_CTYPES[t] for _, t in self.fields)

    @property
    def size(self) -> int:
        return struct.calcsize(self.format)

    def pack(self, **values) -> bytes:
        return struct.pack(self.format, *(values[n] for n, _ in self.fields))

    def unpack(self, payload: bytes) -> dict:
        return dict(zip((n for n, _ in self.fields), struct.unpack_from(self.format, payload)))


def _m(name, id, doc, *fields):
    return Message(name, id, tuple(fields), doc)


RATES = {'RATE_80M': 0, 'RATE_40M': 1, 'RATE_16M': 2}
RATE_HZ = {0: 80_000_000, 1: 40_000_000, 2: 16_000_000}
FORMATS = {'FORMAT_IQ32': 0, 'FORMAT_IQ10': 1, 'FORMAT_IQ8': 2}
WORKERS = {'WORKERS_SINGLE': 0, 'WORKERS_DUAL_ASSIST': 1, 'WORKERS_DUAL': 2}
GPIO_STATES = {'GPIO_FLOAT': 0, 'GPIO_LOW': 1, 'GPIO_HIGH': 2}
RUN_STATUS = {'RUN_OK': 0, 'RUN_FAIL_ARG': 1, 'RUN_FAIL_LATE': 2, 'RUN_FAIL_AGE': 3,
              'RUN_FAIL_START': 4, 'RUN_FAIL_END': 5, 'RUN_FAIL_LENGTH': 6}
ERRORS = {'ERR_NONE': 0, 'ERR_UNKNOWN_COMMAND': 1, 'ERR_BAD_LENGTH': 2, 'ERR_BAD_ARGUMENT': 3,
          'ERR_BUSY': 4, 'ERR_TIMEOUT': 5, 'ERR_UNSUPPORTED': 6, 'ERR_NO_MEMORY': 7, 'ERR_HARDWARE': 8}

REQUESTS = (
    _m('GET_INFO', 0x01, 'Firmware identity and limits; answered with INFO.'),
    _m('RELEASE', 0x02, 'Give up the port lease now.'),
    _m('SET_BAUD', 0x03, 'UART only: ACK at the old rate, then switch.', ('baud', 'uint32_t')),
    _m('SET_FREQUENCY', 0x10, 'Tune the receive LO.', ('mhz', 'uint16_t'), ('reserved', 'uint16_t')),
    _m('SET_PLL_OFFSET', 0x11, 'Fine LO offset applied from the next tune.', ('khz', 'int32_t')),
    _m('SET_GAIN', 0x12, 'manual=0: hardware AGC; manual=1: fixed PHY gain index.',
       ('manual', 'uint8_t'), ('index', 'uint8_t')),
    _m('GET_GAIN', 0x13, 'Answered with GAIN.'),
    _m('SET_FILTER', 0x14, 'Baseband filter code 0..63, or -1 for PHY automatic.', ('code', 'int8_t')),
    _m('SET_BANDWIDTH', 0x15, 'Approximate two-sided bandwidth in MHz (13..69); 0 = widest.',
       ('mhz', 'uint8_t')),
    _m('GET_FILTER', 0x16, 'Answered with FILTER.'),
    _m('SET_DC_MODE', 0x17, 'Spectrum 0 Hz handling: 0 = notch bin 0, 1 = slow tracker.', ('mode', 'uint8_t')),
    _m('SET_WORKERS', 0x18, 'Second-core usage for streams.', ('mode', 'uint8_t')),
    _m('GET_WORKERS', 0x19, 'Answered with WORKERS.'),
    _m('GET_MEMORY', 0x1A, 'Answered with MEMORY.'),
    _m('CAPTURE', 0x20, 'Burst capture, repeated; DATA frames then RUN_DONE.',
       ('rate', 'uint8_t'), ('format', 'uint8_t'), ('reserved', 'uint16_t'),
       ('samples', 'uint32_t'), ('repeats', 'uint32_t')),
    _m('RING_STATS', 0x21, 'Gapless continuous run, statistics only; RUN_DONE. duration 0 = until STOP.',
       ('rate', 'uint8_t'), ('reserved', 'uint8_t'), ('reserved2', 'uint16_t'), ('duration_ms', 'uint32_t')),
    _m('RING_CAPTURE', 0x22, 'Gapless capture of 1..3 consecutive units; RING_DATA then RUN_DONE.',
       ('rate', 'uint8_t'), ('units', 'uint8_t'), ('reserved', 'uint16_t')),
    _m('SPECTRUM', 0x23, 'On-chip FFT stream; SPECTRUM_FRAME (+STATS) then RUN_DONE.',
       ('rate', 'uint8_t'), ('max_hold', 'uint8_t'), ('nfft', 'uint16_t'),
       ('stats', 'uint8_t'), ('reserved', 'uint8_t'), ('reserved2', 'uint16_t'), ('duration_ms', 'uint32_t')),
    _m('IQ_STREAM', 0x24, 'Decimated IQ stream; IQ_FRAME then RUN_DONE. shift 255 = automatic.',
       ('rate', 'uint8_t'), ('bits', 'uint8_t'), ('decimation', 'uint16_t'),
       ('shift', 'uint8_t'), ('rotate', 'uint8_t'), ('reserved', 'uint16_t'), ('duration_ms', 'uint32_t')),
    _m('SCAN', 0x25, 'Full-band scan; SCAN_SEGMENT frames then RUN_DONE. sweeps 0 = until STOP; gain 255 = AGC.',
       ('from_mhz', 'uint16_t'), ('to_mhz', 'uint16_t'), ('rate', 'uint8_t'), ('averages', 'uint8_t'),
       ('nfft', 'uint16_t'), ('gain', 'uint8_t'), ('reserved', 'uint8_t'), ('sweeps', 'uint16_t')),
    _m('STOP', 0x2F, 'Ends a running stream or scan (any byte also does).'),
    _m('GET_GPIO', 0x30, 'Answered with GPIO.'),
    _m('SET_GPIO', 0x31, 'Drive or float a free GPIO.', ('pin', 'uint8_t'), ('state', 'uint8_t')),
)

RESPONSES = (
    _m('ACK', 0x80, 'Result of a request without data.', ('error', 'uint8_t'), ('reserved', 'uint8_t'),
       ('request', 'uint8_t'), ('reserved2', 'uint8_t')),
    _m('INFO', 0x81, 'Followed by the JSON version record (variable length).',
       ('protocol', 'uint16_t'), ('min_mhz', 'uint16_t'), ('max_mhz', 'uint16_t'), ('gain_max', 'uint8_t'),
       ('bw_min_mhz', 'uint8_t'), ('bw_max_mhz', 'uint8_t'), ('rates', 'uint8_t'),
       ('max_samples', 'uint16_t'), ('max_nfft', 'uint16_t'), ('reserved', 'uint16_t')),
    _m('GAIN', 0x82, 'Current gain state.', ('manual', 'uint8_t'), ('index', 'uint8_t'),
       ('max_index', 'uint8_t'), ('forced', 'uint8_t')),
    _m('FILTER', 0x83, 'Requested code (-1 automatic) and the two active register codes.',
       ('code', 'int8_t'), ('active_i', 'uint8_t'), ('active_q', 'uint8_t'), ('reserved', 'uint8_t')),
    _m('WORKERS', 0x84, 'Second-core state.', ('mode', 'uint8_t'), ('core1_alive', 'uint8_t'),
       ('reserved', 'uint16_t'), ('assist_blocks', 'uint32_t')),
    _m('MEMORY', 0x85, 'Internal heap and ring geometry.', ('heap_free', 'uint32_t'),
       ('heap_min_free', 'uint32_t'), ('banks', 'uint16_t'), ('threshold', 'uint16_t')),
    _m('GPIO', 0x86, 'Followed by (pin u8, state u8) pairs for every free pin.'),
    _m('DATA', 0xA0, 'One burst capture; followed by the samples in `format`.',
       ('samples', 'uint32_t'), ('elapsed_us', 'uint32_t'), ('format', 'uint8_t'), ('rate', 'uint8_t'),
       ('reserved', 'uint16_t')),
    _m('RING_DATA', 0xA1, 'Units of a ring capture; followed by raw IQ32 words, back to back.',
       ('rate_hz', 'uint32_t'), ('units', 'uint32_t'), ('count0', 'uint32_t'), ('count1', 'uint32_t'),
       ('count2', 'uint32_t')),
    _m('SCAN_SEGMENT', 0xA6, 'One scan step; followed by `bins` 0.5 dB codes, ascending frequency.',
       ('sweep', 'uint32_t'), ('step', 'uint16_t'), ('steps', 'uint16_t'), ('lo_mhz', 'uint16_t'),
       ('bins', 'uint16_t'), ('start_hz', 'uint32_t'), ('bin_millihz', 'uint32_t'), ('gain', 'uint8_t'),
       ('ffts', 'uint8_t'), ('db_step', 'uint8_t'), ('reserved', 'uint8_t'), ('elapsed_us', 'uint32_t')),
    _m('RUN_DONE', 0xA5, 'Ends every CAPTURE / RING_* / SPECTRUM / IQ_STREAM / SCAN request.',
       ('status', 'uint8_t'), ('stopped_by_host', 'uint8_t'), ('error', 'uint8_t'), ('reserved', 'uint8_t'),
       ('detail', 'uint32_t'), ('units', 'uint32_t'), ('late_max', 'uint32_t'), ('pairs', 'uint64_t'),
       ('elapsed_us', 'uint64_t'), ('work_max', 'uint32_t'), ('frames', 'uint32_t'), ('drops', 'uint32_t'),
       ('abandoned', 'uint32_t'), ('ffts', 'uint32_t'), ('lost_pairs', 'uint32_t')),
)

# Records the library emits from streams (spectrum / IQ / statistics).
RECORDS = (
    _m('SPECTRUM_FRAME', 0xA2, 'Followed by 1 << nfft_log2 bin codes (dB * db_step, natural order).',
       ('frame', 'uint32_t'), ('pair_index', 'uint64_t'), ('pairs', 'uint32_t'), ('ffts', 'uint16_t'),
       ('flags', 'uint8_t'), ('gain', 'uint8_t'), ('drops', 'uint16_t'), ('nfft_log2', 'uint8_t'),
       ('db_step', 'uint8_t')),
    _m('IQ_FRAME', 0xA3, 'Followed by `samples` IQ pairs of `bits` bits each component.',
       ('frame', 'uint32_t'), ('sample_index', 'uint64_t'), ('samples', 'uint16_t'), ('bits', 'uint8_t'),
       ('flags', 'uint8_t'), ('decimation', 'uint16_t'), ('gain', 'uint8_t'), ('shift', 'uint8_t')),
    _m('STATS', 0xA4, 'Live spectrum statistics, about four per second.',
       ('core0_permille', 'uint16_t'), ('core1_permille', 'uint16_t'), ('coverage_permille', 'uint16_t'),
       ('mode', 'uint16_t'), ('heap_free', 'uint32_t'), ('heap_largest', 'uint32_t'),
       ('abandoned', 'uint32_t'), ('drops', 'uint32_t'), ('late_max', 'uint16_t'),
       ('queue_permille', 'uint16_t'), ('ffts_per_s', 'uint32_t')),
)

ALL = REQUESTS + RESPONSES + RECORDS
BY_ID = {m.id: m for m in ALL}
BY_NAME = {m.name: m for m in ALL}
