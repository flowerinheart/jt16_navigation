"""Archive executed sources and installed parameters before collecting results."""
import hashlib
import json
import os
from pathlib import Path


def write_provenance(output, inputs=(), parameters=None):
    root = Path(__file__).resolve().parents[2]
    paths = [*root.glob('integration/scripts/*'), *root.glob('integration/benchmarks/manifests/*.json'),
             *root.glob('integration/config/*'), *root.glob('src/jt16_bringup/config/*'),
             *root.glob('install/jt16_bringup/lib/jt16_bringup/*.py'),
             *root.glob('install/jt16_bringup/share/jt16_bringup/config/*'),
             *root.glob('src/jt16_bringup/src/*.cpp'),
             *root.glob('src/jt16_bringup/include/jt16_bringup/*.hpp'),
             *root.glob('src/jt16_bringup/*plugins.xml'),
             *root.glob('install/jt16_bringup/share/jt16_bringup/*plugins.xml'),
             *root.glob('src/jt16_bringup/CMakeLists.txt'),
             *root.glob('src/jt16_bringup/package.xml'),
             *root.glob('external/runtime/FAST_LIO_Hesai/src/*.cpp')]
    archive = Path(output) / 'executed-inputs'
    if archive.exists():
        raise FileExistsError('use a new candidate directory; do not mix run identities')
    hashes = {}
    for path in paths:
        if not path.is_file():
            continue
        relative = path.relative_to(root)
        data = path.read_bytes()
        target = archive / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[str(relative)] = hashlib.sha256(data).hexdigest()
    input_hashes = {}
    for index, path in enumerate(inputs):
        path = Path(path).resolve()
        data = path.read_bytes()
        target = archive/'run-inputs'/f'{index:02d}-{path.name}'
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(data)
        input_hashes[str(path)] = {'sha256': hashlib.sha256(data).hexdigest(),
                                  'archive': str(target.relative_to(Path(output)))}
    binaries = [*root.glob('install/fast_lio/lib/fast_lio/*'),
                *Path('/usr/lib/python3/dist-packages/gz/transport13').glob('_transport*.so'),
                *Path('/usr/lib/x86_64-linux-gnu').glob('libgz-transport13.so.*'),
                *Path('/usr/lib/x86_64-linux-gnu').glob('libgz-msgs10.so.*'),
                *Path('/opt/ros/jazzy/opt/gz_transport_vendor/lib').glob('libgz-transport13.so.*'),
                *Path('/opt/ros/jazzy/opt/gz_msgs_vendor/lib').glob('libgz-msgs10.so.*'),
                root/'install/jt16_bringup/lib/jt16_bringup/tf_publisher_probe',
                root/'install/jt16_bringup/lib/libjt16_prediction_layer.so',
                root/'install/jt16_bringup/lib/libjt16_timed_obstacle_critic.so',
                *(Path('/opt/ros/jazzy/lib')/name for name in (
                    'libgzserver_component.so', 'libmppi_controller.so', 'libmppi_critics.so', 'libnav2_rotation_shim_controller.so', 'libdwb_core.so',
                    'libdwb_critics.so', 'libstandard_traj_generator.so', 'libsimple_goal_checker.so',
                    'libnav2_costmap_2d_core.so', 'liblayers.so',
                    'libnav2_navfn_planner.so', 'libamcl_core.so', 'libcollision_monitor_core.so'))]
    runtime_hashes = {str(path.relative_to(root) if path.is_relative_to(root) else path): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in binaries if path.is_file()}
    (Path(output) / 'run-provenance.json').write_text(json.dumps({
        'schema': 2, 'sources': hashes, 'sensor_model': 'provisional simulated JT16 geometry; not hardware calibration',
        'runtime_hashes': runtime_hashes,
        'inputs': input_hashes, 'parameters': parameters or {},
        'execution_environment': {key:os.environ.get(key) for key in
                                  ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','GZ_PARTITION','ROS_AUTOMATIC_DISCOVERY_RANGE')},
        'localization_translation_limit_m': .5, 'localization_yaw_limit_rad': .35,
        'covariance_roundoff_tolerance': 1e-9,
        'localization_max_age_s': 1., 'obstacle_max_time_gap_s': .25}, indent=2))


def verify_provenance(output):
    root = Path(__file__).resolve().parents[2]
    identity = json.loads((Path(output)/'run-provenance.json').read_text())
    if identity.get('schema') != 2 or not identity.get('sources'):
        raise ValueError('missing current run identity')
    for relative, expected in identity['sources'].items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'candidate changed: {relative}; use a new output root')
    for relative, expected in identity.get('runtime_hashes',{}).items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'runtime changed: {relative}; use a new output root')
    for path, item in identity.get('inputs',{}).items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError(f'input changed: {path}; use a new output root')


if __name__ == '__main__':
    import sys
    verify_provenance(sys.argv[1])
