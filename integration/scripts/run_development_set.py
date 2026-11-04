#!/usr/bin/env python3
"""Run a mixed development manifest through the existing mapping/navigation tools."""
import argparse
import json
import subprocess
from pathlib import Path

from run_provenance import verify_provenance


def validate_manifest(manifest):
    entries = manifest['entries']
    cases = {(entry['dataset'], entry.get('world_id', entry.get('world')),
              entry.get('obstacles'), entry['seed']) for entry in entries}
    if (len(entries) != manifest['navigation_case_count'] or
            len({entry['id'] for entry in entries}) != len(entries) or
            len(cases) != len(entries) or
            any(entry['dataset'] not in ('BARN', 'Arena') for entry in entries)):
        raise ValueError('invalid development manifest')


def map_world(world, output, scripts, logs):
    run = output / 'barn30' / f'world_{world:03d}' / 'mapping'
    with (logs / f'map-{world}.log').open('w') as stream:
        process = subprocess.run(['bash', str(scripts / 'run_barn_mapping.sh'), str(world), str(run)],
                                 stdout=stream, stderr=subprocess.STDOUT)
    if (run / 'run-provenance.json').exists():
        verify_provenance(run)
    result = run / 'mapping-result.json'
    return process.returncode == 0 and result.exists() and json.loads(result.read_text()).get('map_valid') is True


def navigate_entry(entry, output, scripts, logs, maps):
    if entry['dataset'] == 'BARN':
        world = entry['world_id']
        run = output / 'barn30' / f'world_{world:03d}' / 'navigation' / f"seed_{entry['seed']:02d}"
        command = ['bash', str(scripts / 'run_barn_navigation.sh'), str(world), str(entry['seed']),
                   str(run.parents[1] / 'mapping/nav-map/map.yaml'), str(run)]
        map_ready = maps[world]
    else:
        run = output / 'arena6' / f"{entry['world']}_obs{entry['obstacles']:02d}" / f"seed_{entry['seed']:02d}"
        command = ['bash', str(scripts / 'run_arena_navigation.sh'), entry['world'],
                   str(entry['obstacles']), str(entry['seed']), str(run)]
        map_ready = True
    exit_code = None
    if map_ready:
        with (logs / f"{entry['id']}.log").open('w') as stream:
            process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
        exit_code = process.returncode
    result = run / 'nav2-goal.json'
    if not result.exists():
        run.mkdir(parents=True, exist_ok=True)
        state = 'PROCESS_FAILURE' if map_ready else 'MISSING_MAP'
        result.write_text(json.dumps({'result': state, 'scored_success': False}) + '\n')
    if (run / 'run-provenance.json').exists():
        verify_provenance(run)
    return {'id': entry['id'], 'result_path': str(result.relative_to(output)), 'runner_exit_code': exit_code,
            **json.loads(result.read_text())}


def run_development_set(manifest, output):
    validate_manifest(manifest)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('use a new output directory')
    output.mkdir(parents=True, exist_ok=True)
    scripts = Path(__file__).resolve().parent
    logs = output / 'runner-logs'
    logs.mkdir()
    worlds = sorted({entry['world_id'] for entry in manifest['entries'] if entry['dataset'] == 'BARN'})
    maps = {world: map_world(world, output, scripts, logs) for world in worlds}
    records = [navigate_entry(entry, output, scripts, logs, maps) for entry in manifest['entries']]
    summary = {'manifest': manifest['name'], 'navigation_cases': len(records),
               'strict_successes': sum(row.get('scored_success') is True and row['runner_exit_code'] == 0 for row in records),
               'collisions': sum(bool(row.get('collision')) for row in records),
               'valid_maps': sum(maps.values()), 'mapping_worlds': len(maps), 'entries': records}
    (output / 'development-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summary = run_development_set(json.loads(args.manifest.read_text()), args.output)
    print(json.dumps({key: value for key, value in summary.items() if key != 'entries'}, indent=2))
    raise SystemExit(0 if summary['strict_successes'] == summary['navigation_cases'] and
                     summary['valid_maps'] == summary['mapping_worlds'] and summary['collisions'] == 0 else 1)
