#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PROFILE=${1:-runtime}
BACKEND=${2:-auto}
case "$PROFILE" in runtime|benchmark) ;; *) echo 'usage: setup.sh runtime|benchmark auto|cpu|rocm|cuda' >&2; exit 2;; esac
if [[ "$BACKEND" == auto ]]; then
  if [[ -e /dev/kfd ]]; then BACKEND=rocm
  elif command -v nvidia-smi >/dev/null || [[ -e /dev/nvidiactl || -e /dev/nvhost-gpu ]]; then BACKEND=cuda
  else BACKEND=cpu; fi
fi
case "$BACKEND" in cpu|rocm|cuda) ;; *) echo "Unknown backend: $BACKEND" >&2; exit 2;; esac
source "$ROOT/config/setup/image.env"
ARCH=$(uname -m)
IMAGE=${SETUP_IMAGE:-jt16-deps:jazzy-$PROFILE-$ARCH}
VOLUME=${SETUP_VOLUME:-jt16-workspace-$PROFILE-$ARCH}
JOBS=${BUILD_JOBS:-2}
mkdir -p "$ROOT/.setup"
GPU_ARGS=()
case "$BACKEND" in
  rocm)
    ROCM_ROOT=$(readlink -f "${ROCM_PATH:-/opt/rocm}")
    [[ -e /dev/kfd && -d "$ROCM_ROOT/lib" ]] || { echo 'ROCm requires host /dev/kfd and ROCM_PATH/lib' >&2; exit 2; }
    GPU_ARGS=(--device /dev/kfd --device /dev/dri --mount "type=bind,src=$ROCM_ROOT,dst=$ROCM_ROOT,readonly" -e "ROCM_PATH=$ROCM_ROOT" -e "LD_LIBRARY_PATH=$ROCM_ROOT/lib")
    ;;
  cuda) GPU_ARGS=(--runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility);;
esac
docker build --network host --build-arg HTTP_PROXY --build-arg HTTPS_PROXY --build-arg NO_PROXY --build-arg UBUNTU_MIRROR \
  --build-arg "BASE_IMAGE=$BASE_IMAGE" --build-arg "PROFILE=$PROFILE" \
  -f "$ROOT/docker/Dockerfile" -t "$IMAGE" "$ROOT" 2>&1 | tee "$ROOT/.setup/image-$PROFILE.log"
docker image inspect "$IMAGE" --format '{{.Id}}' > "$ROOT/.setup/image-$PROFILE.id"
docker run --rm --network host "${GPU_ARGS[@]}" \
  --mount "type=bind,src=$ROOT,dst=/source,readonly" --mount "type=volume,src=$VOLUME,dst=/workspace" \
  -e "BUILD_JOBS=$JOBS" -e HTTP_PROXY -e HTTPS_PROXY -e NO_PROXY \
  -e "ROS_DOMAIN_ID=${SETUP_ROS_DOMAIN_ID:-197}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" \
  "$IMAGE" bash /source/scripts/setup_container.sh "$PROFILE" "$BACKEND" \
  2>&1 | tee "$ROOT/.setup/build-$PROFILE-$BACKEND.log"
echo "Setup passed. Workspace volume: $VOLUME; image: $IMAGE"
echo "Open: docker run --rm -it --network host -v $VOLUME:/workspace $IMAGE bash"
