"""Shared locations for JT16 outputs; the repository contains reusable code."""
from datetime import datetime
import os
import hashlib
from pathlib import Path


def artifact_dir(category):
    if category not in ('evaluations', 'experiments', 'reports', 'demos', 'cache'):
        raise ValueError(f'Unknown artifact category: {category}')
    return Path(os.environ.get('JT16_DATA_HOME', '~/.jt16-navigation')).expanduser().resolve() / category


def new_output(category, label):
    parent = artifact_dir(category)
    if category == 'reports':
        parent /= 'evaluation'
    return parent / f'{datetime.now():%Y%m%d-%H%M%S-%f}-{label}'


def prepare_workspace(root):
    """Keep native tool paths readable while fresh-workspace caches live outside Git."""
    root = Path(root).resolve()
    cache = artifact_dir('cache') / 'workspaces' / hashlib.sha256(str(root).encode()).hexdigest()[:12]
    for relative, name in (('build', 'build'), ('install', 'install'), ('log', 'log'),
                           ('integration/models', 'models'), ('integration/worlds', 'worlds'),
                           ('integration/benchmarks/generated', 'benchmark-worlds')):
        legacy = root / relative
        if legacy.exists() or legacy.is_symlink():
            continue
        target = cache / name
        target.mkdir(parents=True, exist_ok=True)
        legacy.parent.mkdir(parents=True, exist_ok=True)
        legacy.symlink_to(target, target_is_directory=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('category', choices=('evaluations', 'experiments', 'reports', 'demos', 'cache'))
    parser.add_argument('--label', help='Use a new timestamped output directory')
    parser.add_argument('--prepare-workspace', type=Path, help='Create compatibility links for missing native cache paths')
    args = parser.parse_args()
    if args.prepare_workspace:
        prepare_workspace(args.prepare_workspace)
    print(new_output(args.category, args.label) if args.label else artifact_dir(args.category))
