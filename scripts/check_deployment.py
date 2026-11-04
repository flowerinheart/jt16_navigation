"""Check installed runtime, optional assets and GPU runtime access without hardware motion."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def gpu_devices(backend):
    if backend == 'cpu':
        return 0
    if backend == 'rocm':
        library = ctypes.CDLL(str(Path(os.environ['ROCM_PATH']) / 'lib/libamdhip64.so'))
        init, count = library.hipInit, library.hipGetDeviceCount
    else:
        library = ctypes.CDLL('libcuda.so.1')
        init, count = library.cuInit, library.cuDeviceGetCount
    assert init(0) == 0, f'{backend} initialization failed'
    devices = ctypes.c_int()
    assert count(ctypes.byref(devices)) == 0 and devices.value > 0, f'No {backend} GPU'
    return devices.value


def check(profile, backend):
    root = Path(__file__).resolve().parents[1]
    prefixes = {}
    for name in ('jt16_bringup', 'fast_lio', 'hesai_ros_driver', 'nav2_bringup'):
        prefixes[name] = subprocess.check_output(['ros2', 'pkg', 'prefix', name], text=True).strip()
    for relative in ('fast_lio/lib/fast_lio/fastlio_mapping',
                     'jt16_bringup/lib/libjt16_prediction_layer.so',
                     'jt16_bringup/lib/libjt16_timed_obstacle_critic.so'):
        path = root / 'install' / relative
        output = subprocess.check_output(['ldd', str(path)], text=True)
        assert 'not found' not in output, output
    if profile == 'benchmark':
        subprocess.run(['gz', 'sim', '--versions'], check=True, stdout=subprocess.DEVNULL)
        for name in ('the-barn-challenge', 'arena-bench'):
            assert (root / 'external/benchmarks' / name).is_dir()
    else:
        assert not (root / 'external/benchmarks').exists(), 'Runtime profile included evaluation datasets'
        simulation = subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_sim'], capture_output=True)
        assert simulation.returncode != 0, 'Runtime profile pulled in Gazebo simulation'
    sources = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in root.glob('dependencies.*.repos')}
    return dict(profile=profile, backend=backend, architecture=platform.machine(),
                gpu_devices=gpu_devices(backend), package_prefixes=prefixes,
                source_manifests=sources, local_extracted_ros_fallback=False)


if __name__ == '__main__':
    print(json.dumps(check(*sys.argv[1:]), indent=2))
