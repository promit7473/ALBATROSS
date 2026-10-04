#!/usr/bin/env python3
"""Read-only startup conflict detection. Never stop processes or open aircraft links."""
import os
from pathlib import Path
import sys


def conflict(stage, argv):
    names = {Path(arg).name for arg in argv}
    if stage in ('1', '2'):
        if 'mav_hub.py' in names or 'rtsp_bridge.py' in names:
            return 'aircraft relay/camera session is still running'
        if stage == '1' and 'rc_to_sitl.py' in names:
            return 'manual RC bridge is still running'
    if stage == '3' and ('rc_to_sitl.py' in names or 'px4' in names):
        return 'simulation or RC-to-simulation session is still running'
    return None


def main(stage):
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            argv = [a.decode(errors='replace') for a in (proc / 'cmdline').read_bytes().split(b'\0') if a]
        except (OSError, PermissionError):
            continue
        reason = conflict(stage, argv)
        if reason:
            found.append((proc.name, reason, ' '.join(Path(a).name for a in argv[:2])))
    if found:
        print('Launch blocked before restarting services: conflicting session detected.', file=sys.stderr)
        for pid, reason, name in found:
            print(f'  PID {pid}: {name} — {reason}', file=sys.stderr)
        print('End the previous aircraft/simulator session intentionally, then launch again. No process was stopped.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1]))
