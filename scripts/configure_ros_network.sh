#!/usr/bin/env bash
set -euo pipefail
IMAGE=${1:?usage: configure_ros_network.sh dependency-image}
# Fragmented LiDAR samples exceed Ubuntu's default UDP receive limit.
docker run --rm -i --network host --cap-add NET_ADMIN \
  --mount type=bind,src=/proc/sys/net/core,dst=/host-net-core \
  --mount type=bind,src=/etc/sysctl.d,dst=/host-sysctl.d \
  "$IMAGE" python3 - <<'PY'
from pathlib import Path

settings = []
for name in ('rmem_max', 'wmem_max'):
    path = Path('/host-net-core') / name
    before = int(path.read_text())
    value = max(before, 4 * 1024 * 1024)
    if before < value:
        path.write_text(f'{value}\n')
    assert int(path.read_text()) >= value, f'Cannot configure net.core.{name}'
    settings.append(f'net.core.{name} = {value}\n')
    print(f'net.core.{name}: {before} -> {value}')
Path('/host-sysctl.d/90-jt16-network.conf').write_text(''.join(settings))
PY
