# ESP-SDR

The ESP32-S3's built-in 2.4 GHz radio as a software-defined receiver: raw I/Q
capture at 16, 40 and 80 Msps, gapless continuous streams, an on-chip FFT
spectrum and a decimated I/Q stream, from a ~$2 board.

* **`components/esp_sdr`** – the library. Radio only: tuning, gain, filter,
  burst and continuous capture, spectrum and I/Q streams. It never touches
  USB, UART or networking; streams hand their records to a sink you provide.
* **`firmware/`** – a ready-made serial firmware built on the library: a
  CRC-checked binary protocol over native USB (and optionally UART0).
* **`host/esp_sdr`** – the protocol definition (single source of truth) and a
  Python client.

## Quick start

```bash
git clone --recursive <repo> && cd <repo>          # esp-dsp is a submodule
. $IDF_PATH/export.sh                               # ESP-IDF 5.0 or newer
idf.py -C firmware set-target esp32s3
idf.py -C firmware -p /dev/ttyACM0 flash
```

```python
import sys; sys.path.insert(0, 'host')
from esp_sdr import Device

with Device('/dev/ttyACM0') as sdr:
    sdr.set_frequency(2437)                 # MHz, 1840..2790
    iq, info = sdr.capture(16380, rate=2)   # complex64, 16 Msps
    for frame in sdr.stream('SPECTRUM', rate=1, nfft=1024, duration_ms=1000):
        if frame.message.name == 'SPECTRUM_FRAME':
            bins = frame.body()             # 0.5 dB codes, FFT order
    freq_hz, db, _ = sdr.scan(1842, 2790)   # whole range, stitched
```

```bash
python3 tools/scan.py /dev/ttyACM0 --rate 2 --out scan   # scan.csv + scan.png
```

## Using the library in your own application

```c
#include "esp_sdr.h"

static IRAM_ATTR size_t my_write(void *ctx, const uint8_t *data, size_t size) { /* non-blocking */ }
static IRAM_ATTR bool my_stop(void *ctx) { /* true to end the run */ }

ESP_ERROR_CHECK(esp_sdr_init());
esp_sdr_set_frequency(2440);
esp_sdr_sink_t sink = {.write = my_write, .stop_requested = my_stop};
esp_sdr_run_config_t run = {.mode = ESP_SDR_RUN_SPECTRUM, .rate = ESP_SDR_RATE_40M,
                            .nfft = 1024, .duration_ms = 1000};
esp_sdr_run_result_t result;
esp_sdr_run(&run, &sink, &result);
```

C++ users can include `esp_sdr.hpp` (`esp_sdr::Radio`, same calls, no exceptions or heap).

Add `components/esp_sdr` and `components/esp-dsp` to `EXTRA_COMPONENT_DIRS` and
copy the essential settings from `firmware/sdkconfig.defaults` (single-core
FreeRTOS, interrupt watchdog off, 240 MHz).

## Capabilities (ESP32-S3)

| | |
|---|---|
| Tuning | 1840–2790 MHz (verified); 2.4 GHz-only silicon, no sub-GHz |
| Burst capture | 256–16 380 samples at 80 / 40 / 16 Msps; IQ32, packed IQ10 or IQ8 |
| Continuous | gapless, sentinel-verified ring at 80 / 40 / 16 Msps |
| Spectrum | 256–2048-point FFT, mean or max-hold, both cores |
| Full-band scan | 1842–2790 MHz in 0.9 s (78 kHz bins), 1.6 s (39 kHz) or 4.3 s (15.6 kHz) |
| I/Q stream | decimation 64–1024, 4/8/16-bit, optional fs/4 shift |
| Bandwidth | baseband filter ≈ 13–69 MHz |

## Repository

```
components/esp_sdr/   library (see docs/architecture.md)
components/esp-dsp/   Espressif esp-dsp (submodule, v1.8.2)
firmware/             serial firmware: link, protocol dispatcher, GPIO
host/esp_sdr/         protocol.py (source of truth), device.py (client)
tools/                scan.py (sweep -> CSV/PNG), gen_protocol.py (headers + docs), version.py
tests/host/           host unit tests: python -m unittest discover -s tests/host
tests/hw/             hardware parity suite: python tests/hw/parity.py /dev/ttyACM0
docs/                 protocol.md, architecture.md, refactor-log.md
```

After editing `host/esp_sdr/protocol.py`, run `python tools/gen_protocol.py`;
CI fails when the generated files are stale.

## Legal

Receiving is generally unrestricted; what you do with received signals may not
be. You are responsible for complying with the radio laws where you operate.
