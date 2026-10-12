# Architecture

## Layers

```
firmware/  (application: owns the transport)
  main.c          init, request loop
  link.c          USB-Serial-JTAG + UART0, port lease, stream sink
  frame.c         frame encode / parse (pure C)
  requests.c      request -> library call -> reply
  gpio_control.c  free-pin control
        |
        v  esp_sdr.h (public API), esp_sdr_record.h (record format)
components/esp_sdr/  (library: radio only)
  sdr.c                 public API, PHY bring-up
  hal/                  ONLY place with undocumented registers / ROM / libphy calls
    phy_private.h       every private PHY / ROM symbol, with provenance
    dump_regs.h         dump engine registers and SRAM map
    rf_frontend.c       tuning, receive mode, gain, baseband filter
    rf_calibration.c    receive-only DC recalibration (link-time wraps)
    burst.c             one-shot capture
    asm/                PIE kernels, core-1 entry
  stream/               continuous runs
    ring.c              bank rotation, sentinel search, deadlines
    spectrum.c          FFT pipeline (single core, core 1, core-0 assist)
    iq_stream.c         two-stage decimating FIR
    worker.c            core-1 bring-up, unit mailbox, bank handshake
    tx_queue.c          output FIFO -> application sink
  dsp/                  pure C, host-tested: LO plan, bandwidth, packing, dB codes, FIR design
  record/               record header encoding
```

## Memory map

The dump engine writes 0x3fcb0000–0x3fce0000: three 64 KiB ring banks; bank 2
doubles as the burst window. `SOC_RESERVE_MEMORY_REGION` keeps the heap out and
`sram_guard.ld` fails the link if `.data`, `.bss` or IRAM grows into it. Large
buffers that would push `.bss` up (twiddles, FIR state, frame buffers) live on
the heap.

## Streams

A run masks interrupts on core 0 for its whole duration. Core 0 polls the
engine's write index, prepares the next bank with sentinels, switches banks at
12 288 pairs and locates each finished unit exactly by binary search, proving
the stream gapless. Between polls it pumps the output queue to the sink and,
when it fits before the next deadline, runs bounded work.

Core 1 is started bare-metal (FreeRTOS is single-core) and processes posted
units. Bank ownership is a Dekker-style handshake through internal SRAM:
core 0 revokes a bank before reusing it and waits while core 1 reads it.
Work that cannot finish before its bank is reused is abandoned and counted,
never waited for.

## ESP-IDF compatibility

The library depends only on components present in every ESP-IDF 5.x and uses
no API newer than 5.0: core-1 release is written with registers, compare-and-set
is inline `s32c1i`, UART access in streams uses `uart_ll`. The private PHY
symbols exist in the S3 `libphy.a` of IDF 5.1 and 5.3; the
`phy_param` offset used by recalibration is hardware-verified on 5.3 only and must be re-checked when Espressif
ships a new S3 PHY library. CI builds against IDF 5.0–5.5.
