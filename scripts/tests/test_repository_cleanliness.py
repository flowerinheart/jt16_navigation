import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('cleanliness', Path(__file__).resolve().parents[1] / 'check_repository_cleanliness.py')
cleanliness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cleanliness)


class RepositoryCleanlinessTests(unittest.TestCase):
    def test_force_added_artifact_rejected_but_rules_and_fixtures_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '-q', directory], check=True)
            (root / '.gitignore').write_text('/build\n/reports\n')
            for name in ('build/cache.json', 'reports/review.md', 'tests/fixture.json', '.codex/rules.md'):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('sample\n')
            subprocess.run(['git', 'add', '-f', '.'], cwd=root, check=True)
            self.assertEqual(cleanliness.violations(root), ['build/cache.json', 'reports/review.md'])
            subprocess.run(['git', 'rm', '--cached', '-q', 'build/cache.json', 'reports/review.md'], cwd=root, check=True)
            self.assertEqual(cleanliness.violations(root), [])
            self.assertTrue((root / 'reports/review.md').exists())
