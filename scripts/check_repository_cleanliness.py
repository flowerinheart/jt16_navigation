#!/usr/bin/env python3
"""Reject tracked local artifacts; .gitignore is the project path policy."""
import subprocess
import sys
from pathlib import Path


def violations(root):
    paths = subprocess.check_output(['git', 'ls-files', '-z'], cwd=root)
    if not paths:
        return []
    result = subprocess.run(
        ['git', 'check-ignore', '--no-index', '--stdin', '-z'],
        cwd=root, input=paths, capture_output=True)
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.decode())
    return sorted(p.decode() for p in result.stdout.split(b'\0') if p)


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    rejected = violations(root)
    if rejected:
        print('Tracked local artifacts: archive and remove from the Git index:', file=sys.stderr)
        print('\n'.join(rejected), file=sys.stderr)
        sys.exit(1)
    print('Repository artifact policy passed.')
