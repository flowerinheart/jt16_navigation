import hashlib
import csv
from pathlib import Path
import tempfile
import unittest

from compare_results import BASELINE, compare


class ComparisonTests(unittest.TestCase):
    def test_baseline_reproduces_counts_without_modifying_original_summary(self):
        original = BASELINE / 'barn30/mapping-summary.json'
        before = hashlib.sha256(original.read_bytes()).digest()
        with tempfile.TemporaryDirectory() as temporary:
            result = compare(BASELINE, Path(temporary))
        self.assertEqual(result['v0'], result['v1'])
        self.assertEqual(len(result['cases']['v0']), 210)
        self.assertEqual(result['cases']['v0'], result['cases']['v1'])
        self.assertTrue(result['v0']['complete'])
        self.assertEqual(result['targets']['barn_successes']['value'], 88)
        self.assertEqual(result['targets']['arena_successes']['value'], 34)
        self.assertEqual(result['targets']['arena_collisions']['value'], 13)
        self.assertEqual(result['targets']['valid_maps']['value'], 27)
        self.assertEqual(result['targets']['process_failures']['value'], 10)
        self.assertFalse(result['performance_targets_met'])
        self.assertEqual(hashlib.sha256(original.read_bytes()).digest(), before)

    def test_missing_records_cannot_pass_performance_targets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = compare(root / 'empty', root / 'report')
            with (root / 'report/failures.csv').open() as stream:
                missing = [row for row in csv.DictReader(stream) if row['version'] == 'v1']
            self.assertEqual(len(missing), 210)
            self.assertTrue(all(row['failure_category'] == 'MISSING_RECORD' for row in missing))
        self.assertFalse(result['v1']['complete'])
        self.assertEqual(result['v1']['navigation']['BARN']['expected_records'], 150)
        self.assertEqual(result['v1']['navigation']['Arena']['present_records'], 0)
        self.assertTrue(all(not check['pass'] for check in result['targets'].values()))


if __name__ == '__main__':
    unittest.main()
