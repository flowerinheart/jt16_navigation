#!/usr/bin/env python3
"""Freeze the lightweight public V0 BARN-30 and Arena-6 validation suite."""
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BARN = ROOT / 'external/benchmarks/the-barn-challenge'
ARENA = ROOT / 'external/benchmarks/arena-bench'
OUTPUT = ROOT / 'integration/benchmarks/manifests'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


# BARN is published in increasing difficulty order. Sample each third at fixed,
# evenly spaced indices so V0 tuning cannot influence scene selection.
world_ids = [*range(0, 100, 11), *range(100, 200, 11), *range(200, 300, 11)]
barn_entries = []
for rank, world_id in enumerate(world_ids):
    world = BARN / f'jackal_helper/worlds/BARN/world_{world_id}.world'
    path = BARN / f'jackal_helper/worlds/BARN/path_files/path_{world_id}.npy'
    barn_entries.append({
        'world_id': world_id,
        'difficulty': ('easy', 'medium', 'difficult')[rank // 10],
        'world': str(world.relative_to(ROOT)),
        'path': str(path.relative_to(ROOT)),
        'world_sha256': digest(world),
        'path_sha256': digest(path),
        'mapping_seed': 0,
        'navigation_seeds': list(range(5)),
        'start_xyyaw_m_rad': [-2.25, 3.0, 1.5707963267948966],
        'goal_xy_m': [-2.25, 13.0],
        'timeout_s': 150.0,
    })

arena_entries = []
for world_name in ('small_warehouse', 'map2', 'map5'):
    for obstacle_count in (5, 10):
        scenario = ARENA / f'simulator_setup/scenarios/{world_name}_obs{obstacle_count:02d}.json'
        map_yaml = ARENA / f'simulator_setup/maps/{world_name}/map.yaml'
        arena_entries.append({
            'world': world_name,
            'obstacle_count': obstacle_count,
            'scenario': str(scenario.relative_to(ROOT)),
            'map': str(map_yaml.relative_to(ROOT)),
            'scenario_sha256': digest(scenario),
            'map_sha256': digest(map_yaml),
            'navigation_seeds': list(range(10)),
            'timeout_s': 150.0,
        })

OUTPUT.mkdir(parents=True, exist_ok=True)
(OUTPUT / 'barn30.json').write_text(json.dumps({
    'dataset': 'BARN', 'source': 'https://github.com/Daffan/the-barn-challenge',
    'revision': 'bf5a226f6088ec96bf0d2dbee3253a8ea6119b83',
    'selection': '10 evenly spaced IDs from each published difficulty third',
    'mapping_runs': 30, 'navigation_runs': 150, 'entries': barn_entries,
}, indent=2) + '\n')
(OUTPUT / 'arena6.json').write_text(json.dumps({
    'dataset': 'Arena-Bench', 'source': 'https://github.com/ignc-research/arena-bench',
    'revision': '519640415718abc7603ff38bc80b2bd22c38b13c',
    'selection': 'paper evaluation worlds x 5/10 dynamic obstacles',
    'navigation_runs': 60, 'entries': arena_entries,
}, indent=2) + '\n')
print(json.dumps({'barn_worlds': len(barn_entries), 'arena_combinations': len(arena_entries)}, indent=2))
