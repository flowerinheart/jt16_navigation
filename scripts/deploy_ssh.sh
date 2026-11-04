#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TARGET=${1:?usage: deploy_ssh.sh user@host [runtime|benchmark] [auto|cpu|rocm|cuda]}
PROFILE=${2:-runtime}
BACKEND=${3:-auto}
case "$PROFILE" in runtime|benchmark) ;; *) exit 2;; esac
case "$BACKEND" in auto|cpu|rocm|cuda) ;; *) exit 2;; esac
cd "$ROOT"
DESTINATION=jt16-navigation-deploy-$(date -u +%Y%m%dT%H%M%SZ)
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$TARGET")
"${SSH[@]}" "mkdir '$DESTINATION'"
# Send repository files, including pending setup changes; never send ignored results or credentials.
git ls-files --cached --others --exclude-standard -z |
  tar --null --files-from - -czf - | "${SSH[@]}" "tar -xzf - -C '$DESTINATION'"
"${SSH[@]}" "bash '$DESTINATION/scripts/setup.sh' '$PROFILE' '$BACKEND'"
echo "Remote checkout: $TARGET:$DESTINATION"
