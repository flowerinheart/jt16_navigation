"""Output defaults and cache links must work from another checkout/user location."""
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from artifact_paths import artifact_dir, new_output, prepare_workspace


class ArtifactPathTests(unittest.TestCase):
    def test_defaults_and_override_keep_categories_outside_repository(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(artifact_dir('evaluations'), Path.home() / '.jt16-navigation/evaluations')
            self.assertEqual(artifact_dir('reports'), Path.home() / '.jt16-navigation/reports')
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'JT16_DATA_HOME': directory}):
            output = new_output('evaluations', 'development')
            self.assertEqual(output.parent, Path(directory) / 'evaluations')
            self.assertTrue(output.name.endswith('-development'))
            self.assertFalse(output.exists())
            self.assertEqual(new_output('reports', 'comparison').parent, Path(directory) / 'reports/evaluation')
            with self.assertRaises(ValueError): artifact_dir('unknown')

    def test_fresh_workspace_links_are_idempotent_and_preserve_existing_data(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'JT16_DATA_HOME': directory + '/data'}):
            root = Path(directory) / 'checkout'; root.mkdir()
            (root / 'log').mkdir(); (root / 'log/existing.txt').write_text('keep')
            prepare_workspace(root); prepare_workspace(root)
            self.assertEqual((root / 'integration/models').resolve().name, 'models')
            self.assertTrue((root / 'integration/models').resolve().is_relative_to(Path(directory) / 'data/cache/workspaces'))
            (root / 'integration/models/robot.sdf').write_text('model')
            self.assertEqual((root / 'integration/models/robot.sdf').read_text(), 'model')
            self.assertEqual((root / 'log/existing.txt').read_text(), 'keep')
            self.assertFalse((root / 'log').is_symlink())


    def test_two_checkouts_have_independent_cmake_and_model_caches(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'JT16_DATA_HOME': directory + '/data'}):
            roots = [Path(directory) / name for name in ('checkout-a', 'checkout-b')]
            for root in roots:
                root.mkdir()
                (root / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(probe NONE)\n')
                prepare_workspace(root)
                subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], check=True, capture_output=True)
            for relative in ('build', 'install', 'log', 'integration/models', 'integration/worlds', 'integration/benchmarks/generated'):
                self.assertNotEqual((roots[0] / relative).resolve(), (roots[1] / relative).resolve())
            (roots[0] / 'integration/models/robot.sdf').write_text('a')
            self.assertFalse((roots[1] / 'integration/models/robot.sdf').exists())


if __name__ == '__main__': unittest.main()
