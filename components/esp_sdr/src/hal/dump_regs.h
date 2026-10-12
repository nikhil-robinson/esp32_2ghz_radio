/*
 * ESP32-S3 ADC dump engine and the SRAM it writes.
 *
 * The baseband "dump" engine copies raw IQ words into internal SRAM. In
 * one-shot mode it fills the burst window; in circular mode it writes a
 * 16384-pair ring into whichever bank is selected. Register meanings were
 * established on hardware (rates checked against a B210).
 */
#pragma once

#include <stdint.h>

#define DUMP_CTRL_REG 0x60033d5cu
#define DUMP_CTRL_RUN (1u << 31)
#define DUMP_CTRL_START (1u << 19)     /* one-shot start pulse */
#define DUMP_CTRL_DONE (1u << 18)      /* one-shot finished */
#define DUMP_CTRL_RATE_16M (1u << 16)  /* 16 Msps hardware path */
#define DUMP_CTRL_RATE_40M (1u << 15)  /* halves 80 Msps */
#define DUMP_CTRL_CIRCULAR 0x00024000u /* circular 16384-pair ring, IQ source 0 */

#define DUMP_WRITE_INDEX_REG 0x60033d60u

#define DUMP_CONFIG_REG 0x60033d90u
#define DUMP_CONFIG_IQ 0x000c2040u     /* bit fields 0,1,2,3 -> I/Q packing order */

/* SRAM owner / bank select: low 4 bits pick the banks the engine writes. */
#define DUMP_BANK_SELECT_REG 0x600c101cu
#define DUMP_BANK_MASK 15u

/* Burst window = bank 2. */
#define DUMP_BURST_ADDR 0x3fcd0000u
#define DUMP_BURST_BANK_MASK 4u

/* Continuous ring: three 64 KiB banks; bank 3 holds ROM data. */
#define DUMP_RING_BASE 0x3fcb0000u
#define DUMP_RING_END 0x3fce0000u
#define DUMP_RING_STRIDE 0x10000u
#define DUMP_RING_BANKS 3u
#define DUMP_RING_PAIRS 16384u
#define DUMP_RING_MASK (DUMP_RING_PAIRS - 1u)
#define DUMP_RING_THRESHOLD 12288u     /* switch banks after this many pairs */

_Static_assert(DUMP_RING_BASE + DUMP_RING_BANKS * DUMP_RING_STRIDE == DUMP_RING_END, "bank map");
_Static_assert(DUMP_BURST_ADDR == DUMP_RING_BASE + 2u * DUMP_RING_STRIDE, "burst window is bank 2");

static inline uint32_t *dump_bank(unsigned bank)
{
    return (uint32_t *)(DUMP_RING_BASE + bank * DUMP_RING_STRIDE);
}
