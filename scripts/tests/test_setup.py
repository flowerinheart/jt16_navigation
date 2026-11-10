import importlib.util
import json
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('check_deployment', ROOT/'scripts/check_deployment.py')
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)


class SetupTests(unittest.TestCase):

    def test_all_build_commands_inherit_the_requested_job_limit(self):
        script = ROOT / 'scripts/build.sh'
        source = script.read_text()
        prefix = source.split('set +u', 1)[0]
        commands = [line.strip() for line in source.splitlines() if line.strip().startswith('colcon build')]
        stub = "\ncolcon() { /usr/bin/python3 -c 'import os; print(os.environ[\"CMAKE_BUILD_PARALLEL_LEVEL\"], os.environ[\"MAKEFLAGS\"])'; }\n"
        result = subprocess.run(['bash', '-c', prefix + stub + '\n'.join(commands), str(script)],
                                env={**os.environ, 'BUILD_JOBS': '3'}, capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout.splitlines(), ['3 -j3'] * len(commands))


    def test_dependency_patch_is_idempotent_and_rejects_incompatible_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'scripts').mkdir()
            (root/'patches').mkdir()
            repo = root/'external/runtime/FAST_LIO_Hesai'
            (repo/'src').mkdir(parents=True)
            shutil.copyfile(ROOT/'scripts/apply_dependency_patches.sh', root/'scripts/apply_dependency_patches.sh')
            patch_file = ROOT/'patches/fast-lio.patch'
            shutil.copyfile(patch_file, root/'patches'/patch_file.name)
            original = subprocess.check_output(['git', '-C', str(ROOT/'external/runtime/FAST_LIO_Hesai'), 'show', 'HEAD:src/preprocess.cpp'])
            source = repo/'src/preprocess.cpp'
            source.write_bytes(original)
            (repo/'src/laserMapping.cpp').write_bytes(subprocess.check_output(['git', '-C', str(ROOT/'external/runtime/FAST_LIO_Hesai'), 'show', 'HEAD:src/laserMapping.cpp']))
            subprocess.run(['git', 'init', '-q', str(repo)], check=True)
            nav_repo = root/'external/runtime/navigation2'
            nav_source = nav_repo/'nav2_mppi_controller/src/critics/cost_critic.cpp'
            nav_source.parent.mkdir(parents=True)
            nav_original = None
            for relative in ('nav2_mppi_controller/src/optimizer.cpp', 'nav2_mppi_controller/src/critics/cost_critic.cpp', 'nav2_smac_planner/include/nav2_smac_planner/utils.hpp', 'nav2_smac_planner/src/collision_checker.cpp'):
                target = nav_repo/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                original_file = subprocess.check_output(['git', '-C', str(ROOT/'external/runtime/navigation2'), 'show', 'HEAD:'+relative])
                target.write_bytes(original_file)
                if target == nav_source:
                    nav_original = original_file
            shutil.copyfile(ROOT/'patches/nav2-mppi.patch', root/'patches/nav2-mppi.patch')
            subprocess.run(['git', 'init', '-q', str(nav_repo)], check=True)
            command = ['bash', str(root/'scripts/apply_dependency_patches.sh')]
            subprocess.run(command, check=True)
            nav_applied = nav_source.read_bytes()
            self.assertNotEqual(nav_applied, nav_original)
            applied = source.read_bytes()
            mapping = (repo/'src/laserMapping.cpp').read_bytes()
            self.assertNotEqual(applied, original)
            subprocess.run(command, check=True)
            self.assertEqual(source.read_bytes(), applied)
            self.assertEqual(nav_source.read_bytes(), nav_applied)
            source.write_text('incompatible source\n')
            result = subprocess.run(command, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(source.read_text(), 'incompatible source\n')
            self.assertEqual((repo/'src/laserMapping.cpp').read_bytes(), mapping)
            source.write_bytes(applied)
            nav_source.write_text('incompatible native source\n')
            result = subprocess.run(command, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(nav_source.read_text(), 'incompatible native source\n')

    def test_profiles_pin_sources_and_exclude_unused_runtime_repos(self):
        deploy = yaml.safe_load((ROOT/'dependencies.deploy.repos').read_text())['repositories']
        benchmark = yaml.safe_load((ROOT/'dependencies.benchmark.repos').read_text())['repositories']
        self.assertEqual(len(deploy), 3)
        self.assertEqual(len(benchmark), 3)
        for path, item in {**deploy, **benchmark}.items():
            self.assertRegex(item['version'], r'^[0-9a-f]{40}$')
            self.assertEqual(item['type'], 'git')
        self.assertTrue(all('/runtime/' in p for p in deploy))
        self.assertEqual(sum('/benchmarks/' in p for p in benchmark), 2)
        self.assertIn('external/reference/XLeRobot', benchmark)
        self.assertNotIn('ros-jazzy-ros-gz', (ROOT/'config/setup/runtime.apt').read_text())
        self.assertNotIn('ros-jazzy-nav2-bringup', (ROOT/'config/setup/runtime.apt').read_text())

    def test_cpu_setup_uses_isolated_volume_and_readonly_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'repo'
            (root/'scripts').mkdir(parents=True)
            (root/'config/setup').mkdir(parents=True)
            shutil.copyfile(ROOT/'scripts/setup.sh', root/'scripts/setup.sh')
            shutil.copyfile(ROOT/'scripts/configure_ros_network.sh', root/'scripts/configure_ros_network.sh')
            shutil.copyfile(ROOT/'config/setup/image.env', root/'config/setup/image.env')
            docker = Path(directory)/'docker'
            docker.write_text('#!/usr/bin/python3\nimport sys,json\nprint(json.dumps(sys.argv[1:]))\n')
            docker.chmod(0o755)
            env = {**os.environ, 'PATH':directory+':'+os.environ['PATH'], 'SETUP_VOLUME':'jt16-test-isolation'}
            output = subprocess.check_output(['bash', str(root/'scripts/setup.sh'), 'runtime', 'cpu'], env=env, text=True)
            commands = [json.loads(line) for line in output.splitlines() if line.startswith('[')]
            network = next(c for c in commands if '--cap-add' in c)
            self.assertEqual(network[network.index('--cap-add') + 1], 'NET_ADMIN')
            self.assertNotIn('--privileged', network)
            run = next(c for c in commands if c[0]=='run' and '--cap-add' not in c)
            self.assertIn('type=volume,src=jt16-test-isolation,dst=/workspace', run)
            self.assertIn(f'type=bind,src={root},dst=/source,readonly', run)
            self.assertNotIn('--privileged', run)
            self.assertNotIn('--gpus', run)
            self.assertIn('setup_container.sh', ' '.join(run))
            output = subprocess.check_output(['bash', str(root/'scripts/setup.sh'), 'runtime', 'cuda'], env=env, text=True)
            commands = [json.loads(line) for line in output.splitlines() if line.startswith('[')]
            run = next(c for c in commands if c[0]=='run' and '--cap-add' not in c)
            self.assertIn('--runtime=nvidia', run)
            self.assertEqual(run[run.index('--gpus') + 1], 'all')

    def test_unknown_profile_fails_before_docker(self):
        result = subprocess.run(['bash', str(ROOT/'scripts/setup.sh'), 'invalid', 'cpu'], capture_output=True)
        self.assertEqual(result.returncode, 2)

    def test_gpu_initialization_failure_is_not_reported_as_success(self):
        with patch.object(deployment.ctypes, 'CDLL') as library:
            library.return_value.cuInit.return_value = 1
            with self.assertRaisesRegex(AssertionError, 'initialization failed'):
                deployment.gpu_devices('cuda')
        self.assertEqual(deployment.gpu_devices('cpu'), 0)


if __name__ == '__main__':
    unittest.main()
