"""Verify the exact release contents, without running a simulation."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    report = json.loads((root / 'RELEASE_CONTENTS.json').read_text())
    for row in report['files']:
        path = root / row['path']
        data = path.read_bytes()
        if len(data) != row['bytes'] or hashlib.sha256(data).hexdigest() != row['sha256']:
            raise SystemExit('Release mismatch: ' + row['path'])
    from usv_uuc_cooperation.main import assert_frozen_core
    assert_frozen_core()
    print('release PASS:', len(report['files']), 'files;', report['framework'])


if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    main()
