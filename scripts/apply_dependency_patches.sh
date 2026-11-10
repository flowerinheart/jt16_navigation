#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
for item in FAST_LIO_Hesai:fast-lio navigation2:nav2-mppi; do
  repo="$ROOT/external/runtime/${item%%:*}"
  patch="$ROOT/patches/${item##*:}.patch"
  if [ ! -d "$repo" ] && [ "${item%%:*}" = navigation2 ]; then
    continue
  fi
  if git -C "$repo" apply --reverse --check "$patch" 2>/dev/null; then
    continue
  fi
  git -C "$repo" apply --check "$patch"
  git -C "$repo" apply "$patch"
done
