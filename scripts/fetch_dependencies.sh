#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
role=${1:-runtime}
case "$role" in deploy|benchmark|runtime|reference|templates) ;; *) echo "usage: $0 deploy|benchmark|runtime|reference|templates" >&2; exit 2;; esac
vcs import --recursive . < "dependencies.$role.repos"
# Initialize declared repositories without relying on non-existent `vcs list`.
while IFS= read -r repo; do
  git -C "$repo" submodule update --init --recursive
done < <(/usr/bin/python3 - "$role" <<'PYTHON'
import sys,yaml
for path in yaml.safe_load(open('dependencies.'+sys.argv[1]+'.repos'))['repositories']:
    # Upstream reference workspace lacks .gitmodules; it is read-only reference.
    if not path.endswith('/Lidar_nav2_ws'):
        print(path)
PYTHON
)
