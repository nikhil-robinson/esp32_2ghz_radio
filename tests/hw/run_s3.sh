#!/usr/bin/env bash
# Build with the installed ESP-IDF (5.3 OK via idf53_compat.h), flash the S3, run the smoke test.
# Usage: tests/hw/run_s3.sh [/dev/ttyACM0]   (source ESP-IDF export.sh first)
set -euo pipefail
cd "$(dirname "$0")/../.."
PORT=${1:-/dev/ttyACM0}
B=build-hw-s3
idf.py -B "$B" -DIDF_TARGET=esp32s3 -DSDKCONFIG="$B/sdkconfig" \
    "-DCMAKE_C_FLAGS=-include $PWD/tests/hw/idf53_compat.h" -p "$PORT" build flash
python3 tests/hw/s3_smoke.py "$PORT"
