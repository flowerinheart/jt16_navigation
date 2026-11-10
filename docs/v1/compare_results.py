"""Compare the frozen manifests without modifying either result directory."""
import argparse
import ast
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'integration/scripts'))
from artifact_paths import artifact_dir, new_output
BASELINE = artifact_dir('evaluations') / 'v0/full-benchmark-20261101'


def grading_fingerprint(revision=None):
    parts = []
    for filename in ('navigation_metrics.py', 'benchmark_geometry.py'):
        relative = f'integration/scripts/{filename}'
        source = subprocess.check_output(['git', 'show', f'{revision}:{relative}'], cwd=ROOT).decode() if revision else (ROOT / relative).read_text()
        if filename == 'benchmark_geometry.py':
            names = {'base_pose', 'nearest_time_index', 'dynamic_clearances',
                     'static_collision', 'covariance_valid', 'localization_metrics'}
            source = '\n'.join(ast.dump(node) for node in ast.parse(source).body
                               if isinstance(node, ast.FunctionDef) and node.name in names)
        parts.append(source)
    return hashlib.sha256('\n'.join(parts).encode()).hexdigest()


def navigation_records(root, dataset, manifest):
    records = []
    for entry in manifest['entries']:
        directory = (Path('barn30') / f"world_{entry['world_id']:03d}" / 'navigation'
                     if dataset == 'BARN' else Path('arena6') / f"{entry['world']}_obs{entry['obstacle_count']:02d}")
        for seed in entry['navigation_seeds']:
            relative = directory / f'seed_{seed:02d}' / 'nav2-goal.json'
            path = root / relative
            result = json.loads(path.read_text()) if path.exists() else {'result': 'MISSING_RECORD', 'scored_success': False}
            records.append({'dataset': dataset, 'case': str(relative.parent),
                            'present': path.exists(), 'result_sha256': hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None,
                            **result})
    return records


def navigation_summary(records):
    summary = {'expected_records': len(records), 'present_records': sum(row['present'] for row in records),
               'strict_successes': sum(row.get('scored_success') is True for row in records),
               'collisions': sum(row.get('collision') is True for row in records),
               'terminal_states': dict(Counter(row.get('result', 'MISSING') for row in records))}
    for name in ('elapsed_s', 'minimum_obstacle_clearance_m', 'final_goal_distance_m',
                 'localization_translation_error_m', 'localization_yaw_error_rad', 'path_ratio'):
        values = [row[name] for row in records if row.get(name) is not None and math.isfinite(row[name])]
        summary[name] = {'count': len(values), 'missing_count': len(records) - len(values),
                         'mean': sum(values) / len(values) if values else None}
    return summary


def mapping_summary(root):
    with tempfile.TemporaryDirectory(prefix='jt16-v1-score-') as temporary:
        output = Path(temporary)
        for world in sorted((root / 'barn30').glob('world_*')):
            (output / world.name).symlink_to(world.resolve(), target_is_directory=True)
        subprocess.run(['/usr/bin/python3', str(ROOT / 'integration/scripts/score_v0.py'),
                        '--results', str(output), '--mapping-only'], check=True, stdout=subprocess.DEVNULL)
        return json.loads((output / 'mapping-summary.json').read_text())


def batch_summary(root):
    records = []
    navigation = {}
    for dataset, filename in (('BARN', 'barn30.json'), ('Arena', 'arena6.json')):
        manifest = json.loads((ROOT / 'integration/benchmarks/manifests' / filename).read_text())
        rows = navigation_records(root, dataset, manifest)
        navigation[dataset] = navigation_summary(rows)
        records.extend(rows)
    mapping = mapping_summary(root)
    return {'root': str(root), 'mapping': mapping, 'navigation': navigation,
            'complete': mapping['mapping_runs_scored'] == 30 and all(row['present'] for row in records)}, records


def failure_category(record):
    categories = []
    if record.get('result') != 'SUCCEEDED':
        categories.append(record.get('result', 'MISSING'))
    for key, category in (('collision', 'COLLISION'), ('input_failure', 'INPUT_FAILURE')):
        if record.get(key):
            categories.append(category)
    for key, category in (('scoring_valid', 'SCORING_INPUT'), ('localization_success', 'LOCALIZATION')):
        if record.get(key) is False:
            categories.append(category)
    return ';'.join(categories) or 'STRICT_GOAL'


def failure_rows(version, records):
    rows = []
    for record in records:
        if record.get('scored_success') is True:
            continue
        rows.append({'version': version, 'dataset': record['dataset'], 'case': record['case'],
                     'result': record.get('result'), 'failure_category': failure_category(record),
                     'collision': record.get('collision'),
                     'localization_success': record.get('localization_success'),
                     'input_failure': record.get('input_failure'), 'scoring_error': record.get('scoring_error'),
                     'elapsed_s': record.get('elapsed_s'), 'goal_distance_m': record.get('final_goal_distance_m'),
                     'clearance_m': record.get('minimum_obstacle_clearance_m'), 'result_sha256': record['result_sha256']})
    return rows


def compare(v1_root, output):
    fingerprint = grading_fingerprint()
    if fingerprint != grading_fingerprint('v0'):
        raise ValueError('navigation grading changed since V0')
    v0, old_records = batch_summary(BASELINE)
    v1, new_records = batch_summary(v1_root)
    nav = v1['navigation']
    process_failures = sum(row.get('result') == 'PROCESS_FAILURE' for row in new_records)
    checks = {'barn_successes': {'value': nav['BARN']['strict_successes'], 'minimum': 105},
              'arena_successes': {'value': nav['Arena']['strict_successes'], 'minimum': 42},
              'arena_collisions': {'value': nav['Arena']['collisions'], 'maximum': 6},
              'valid_maps': {'value': v1['mapping']['valid_mapping_runs'], 'minimum': 30},
              'process_failures': {'value': process_failures, 'maximum': 4}}
    for check in checks.values():
        check['pass'] = v1['complete'] and check['value'] >= check.get('minimum', -math.inf) and check['value'] <= check.get('maximum', math.inf)
    comparison = {'schema': 1, 'grading_fingerprint': fingerprint, 'v0': v0, 'v1': v1,
                  'cases': {'v0': old_records, 'v1': new_records},
                  'targets': checks, 'performance_targets_met': all(check['pass'] for check in checks.values())}
    output.mkdir(parents=True, exist_ok=True)
    (output / 'comparison.json').write_text(json.dumps(comparison, indent=2, allow_nan=False) + '\n')
    failures = failure_rows('v0', old_records) + failure_rows('v1', new_records)
    with (output / 'failures.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['version', 'dataset', 'case', 'result', 'failure_category', 'collision',
                                'localization_success', 'input_failure', 'scoring_error', 'elapsed_s',
                                'goal_distance_m', 'clearance_m', 'result_sha256'])
        writer.writeheader()
        writer.writerows(failures)
    return comparison


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--v1', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=new_output('reports', 'comparison'))
    args = parser.parse_args()
    result = compare(args.v1.resolve(), args.output.resolve())
    print(json.dumps(result['targets'], indent=2))
