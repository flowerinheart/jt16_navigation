#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PROFILE=${1:-runtime}
case "$PROFILE" in runtime|benchmark) ;; *) echo 'usage: install_dependencies.sh runtime|benchmark' >&2; exit 2;; esac
source "$ROOT/config/setup/image.env"
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo 'Ubuntu 24.04 required' >&2; exit 2; }
[[ $(id -u) == 0 ]] || { echo 'Run dependency installation as root inside the container' >&2; exit 2; }
if [[ -n "${UBUNTU_MIRROR:-}" ]]; then
  sed -E -i "s#http://(archive.ubuntu.com/ubuntu|security.ubuntu.com/ubuntu|ports.ubuntu.com/ubuntu-ports)/?#$UBUNTU_MIRROR#g" /etc/apt/sources.list.d/ubuntu.sources
fi
sed -i 's/ noble-backports//g' /etc/apt/sources.list.d/ubuntu.sources
rm -f /etc/apt/apt.conf.d/docker-clean
apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update
apt-get install -y --no-install-recommends ca-certificates curl gnupg
install -d /etc/apt/keyrings
curl -fsSL --retry 3 "https://raw.githubusercontent.com/ros/rosdistro/$ROSDISTRO_COMMIT/ros.key" |
  gpg --batch --yes --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update
mapfile -t packages < "$ROOT/config/setup/runtime.apt"
if [[ "$PROFILE" == benchmark ]]; then mapfile -t optional < "$ROOT/config/setup/benchmark.apt"; packages+=("${optional[@]}"); fi
for attempt in 1 2 3; do
  if apt-get -o Acquire::Retries=3 install -y --no-install-recommends "${packages[@]}"; then break; fi
  [[ "$attempt" != 3 ]] || exit 1
done
mkdir -p /opt/jt16-setup/evidence
dpkg-query -W -f='${Package}\t${Version}\n' > /opt/jt16-setup/evidence/system-packages.tsv
cp /etc/apt/sources.list.d/ubuntu.sources /opt/jt16-setup/evidence/
rm -rf /var/lib/apt/lists/*
