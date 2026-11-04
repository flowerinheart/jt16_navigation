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
