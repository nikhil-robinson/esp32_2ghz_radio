#!/usr/bin/env python3
"""Write the firmware version record (git revision + UTC build time) as a C header."""
import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def revision(root: Path) -> str:
    try:
        rev = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no'],
                                        text=True)
        return rev + ('-dirty' if dirty else '')
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    record = json.dumps({'revision': revision(args.root), 'built': stamp, 'target': 'esp32s3'}, separators=(',', ':'))
    text = f'#pragma once\n#define FIRMWARE_VERSION_JSON {json.dumps(record)}\n'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists() or args.output.read_text() != text:
        args.output.write_text(text)


if __name__ == '__main__':
    main()
