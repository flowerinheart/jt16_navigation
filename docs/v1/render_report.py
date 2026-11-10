"""Render frozen full-batch comparisons and a source-verified Dev16 telemetry replay."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Circle, Polygon
import numpy as np
import yaml

from compare_results import BASELINE, ROOT, compare
from artifact_paths import new_output
sys.path.insert(0, str(ROOT))
from docs.v0.render_report import map_panel
from integration.scripts.navigation_metrics import local_trajectory
from integration.scripts.run_provenance import verify_provenance

CASES = [('Warehouse (10)', 'arena6/small_warehouse_obs10/seed_00'),
         ('Map2 (5)', 'arena6/map2_obs05/seed_00'),
         ('Map5 (10)', 'arena6/map5_obs10/seed_00')]


def load_navigation(root, name, relative, verify=False):
    path = root / relative
    archive_path = path / 'navigation-trajectories.npz'
    if not archive_path.exists():
        return None
    if verify:
        verify_provenance(path)
    summary = json.loads((path / 'nav2-goal.json').read_text())
    identity = json.loads((path / 'run-provenance.json').read_text())
    with np.load(archive_path) as archive:
        gt, amcl, dynamic = (archive[key].copy() for key in ('gt', 'amcl', 'dynamic'))
    start, end = summary['mission_start_stamp_s'], summary['mission_end_stamp_s']
    gt = gt[(gt[:, 3] >= start) & (gt[:, 3] <= end)]
    assert len(gt) >= 2 and np.isfinite(gt).all()
    parameters = yaml.safe_load((ROOT / identity['parameters']['nav2_params_file']).read_text())
    footprint = np.asarray(ast.literal_eval(parameters['local_costmap']['local_costmap']['ros__parameters']['footprint']))
    assert footprint.shape == (4, 2) and np.isfinite(footprint).all()
    return dict(name=name, path=path, summary=summary, identity=identity, gt=gt,
                amcl=amcl, dynamic=dynamic, footprint=footprint)


def navigation_figure(full, output):
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), constrained_layout=True)
    for row, (version, root) in enumerate((('V0', BASELINE), ('V1', full))):
        for column, (name, relative) in enumerate(CASES):
            ax = axes[row, column]
            case = load_navigation(root, name, relative, version == 'V1')
            if case is None:
                ax.text(.5, .5, f'{version}: no navigation trajectory', ha='center', transform=ax.transAxes)
                ax.axis('off')
                continue
            map_panel(ax, case)
            gt = case['gt']
            amcl = case['amcl']
            amcl = amcl[(amcl[:, 3] >= gt[0, 3]) & (amcl[:, 3] <= gt[-1, 3])]
            ax.plot(gt[:, 0], gt[:, 1], color='#007f86', label='Ground truth')
            ax.plot(amcl[:, 0], amcl[:, 1], color='#d46a36', alpha=.8, label='AMCL')
            ax.scatter(*case['summary']['gt_goal_xy_m'], marker='*', color='#204c80')
            result = case['summary']
            status = 'PASS' if result['scored_success'] else 'FAIL'
            ax.set_title(f"{version} {name} / {status}\n{result['elapsed_s']:.1f}s | gap {result['minimum_obstacle_clearance_m']:.3f}m")
    axes[0, 0].legend(fontsize=8)
    fig.savefig(output / 'navigation-comparison.png', dpi=160)
    plt.close(fig)


def mapping_figure(full, output):
    manifest = json.loads((ROOT / 'integration/benchmarks/manifests/barn30.json').read_text())
    origins = {entry['world_id']: entry['start_xyyaw_m_rad'] for entry in manifest['entries']}
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), constrained_layout=True)
    for row, (version, root) in enumerate((('V0', BASELINE), ('V1', full))):
        for column, world in enumerate((22, 144, 277)):
            path = root / 'barn30' / f'world_{world:03d}' / 'mapping'
            ax = axes[row, column]
            if not (path / 'nav-map/map.yaml').exists():
                ax.text(.5, .5, f'{version} BARN{world}\nNo saved map', ha='center', transform=ax.transAxes)
                ax.axis('off')
                continue
            if version == 'V1':
                verify_provenance(path)
            with np.load(path / 'mapping-trajectories.npz') as archive:
                gt, lio = archive['gt'], archive['lio']
            x, y, yaw = origins[world]
            origin = SimpleNamespace(initial_x=x, initial_y=y, initial_yaw=yaw,
                                     ground_truth_origin_x=None, ground_truth_origin_y=None, ground_truth_origin_yaw=None)
            truth = np.asarray(local_trajectory(gt[:, [1, 2, 3, 0]].tolist(), origin))
            case = dict(gt=truth, identity={'parameters': {'map_yaml': str(path / 'nav-map/map.yaml')}})
            map_panel(ax, case)
            ax.plot(truth[:, 0], truth[:, 1], color='#007f86', label='Ground truth')
            ax.plot(lio[:, 1], lio[:, 2], color='#d46a36', label='FAST-LIO')
            result = json.loads((path / 'mapping-result.json').read_text())
            metrics = json.loads((path / 'map-metrics.json').read_text())
            ax.set_title(f"{version} BARN{world}\nATE {result['ate_rmse_m']:.3f}m | F1 {metrics['occupied_f1']:.3f}")
    axes[0, 0].legend(fontsize=8)
    fig.savefig(output / 'mapping-comparison.png', dpi=160)
    plt.close(fig)


def metrics_figure(comparison, output):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), constrained_layout=True)
    for ax, dataset, total in zip(axes[:2], ('BARN', 'Arena'), (150, 60)):
        values = [comparison[version]['navigation'][dataset]['strict_successes'] for version in ('v0', 'v1')]
        bars = ax.bar(('V0', 'V1'), values, color=('#d46a36', '#007f86'))
        ax.bar_label(bars)
        ax.axhline(.7 * total, color='#204c80', ls='--', label='V1 target')
        ax.set(ylim=(0, total), title=f'{dataset}: strict successes / {total}')
        ax.legend(fontsize=8)
    collisions = [comparison[version]['navigation']['Arena']['collisions'] for version in ('v0', 'v1')]
    bars = axes[2].bar(('V0', 'V1'), collisions, color=('#d46a36', '#007f86'))
    axes[2].bar_label(bars)
    axes[2].axhline(6, color='#204c80', ls='--', label='Maximum: 6')
    axes[2].set_title('Arena collisions / 60')
    axes[2].legend(fontsize=8)
    fig.savefig(output / 'full-benchmark.png', dpi=160)
    plt.close(fig)


def replay(dev16, output):
    cases = [load_navigation(dev16, name, relative, True) for name, relative in CASES]
    assert all(case and case['summary']['scored_success'] for case in cases)
    fig, axes = plt.subplots(1, 3, figsize=(12, 5), constrained_layout=True)
    artists = []
    for ax, case in zip(axes, cases):
        map_panel(ax, case)
        line, = ax.plot([], [], color='#007f86', lw=2)
        robot = Polygon(case['footprint'], color='#007f86')
        ax.add_patch(robot)
        obstacles = {}
        for agent in np.unique(case['dynamic'][:, 1]):
            samples = case['dynamic'][case['dynamic'][:, 1] == agent]
            circle = Circle(samples[0, 2:4], samples[0, 4], color='#d46a36', alpha=.75)
            ax.add_patch(circle)
            obstacles[agent] = (circle, samples)
        artists.append((line, robot, obstacles))
    duration = max(case['summary']['elapsed_s'] for case in cases)
    fps, speed = 12, 8

    def frame(index):
        elapsed = min(index * speed / fps, duration)
        for ax, case, (line, robot, obstacles) in zip(axes, cases, artists):
            result = case['summary']
            stamp = result['mission_start_stamp_s'] + min(elapsed, result['elapsed_s'])
            gt = case['gt']
            row = np.clip(np.searchsorted(gt[:, 3], stamp, side='right') - 1, 0, len(gt) - 1)
            x, y, yaw = gt[row, :3]
            c, s = math.cos(yaw), math.sin(yaw)
            robot.set_xy(case['footprint'] @ np.asarray([[c, s], [-s, c]]) + [x, y])
            line.set_data(gt[:row + 1, 0], gt[:row + 1, 1])
            for circle, samples in obstacles.values():
                observation = samples[np.abs(samples[:, 0] - stamp).argmin()]
                assert abs(observation[0] - stamp) <= .25
                circle.center = tuple(observation[2:4])
                circle.set_radius(observation[4])
            ax.set_title(f"{case['name']} / t={min(elapsed, result['elapsed_s']):.1f}s")
        fig.suptitle('V1 Dev16 / telemetry replay / 8x')

    animation = FuncAnimation(fig, frame, frames=math.ceil(duration * fps / speed) + 1)
    frame(0)
    fig.savefig(output / 'replay-poster.png', dpi=120)
    animation.save(output / 'arena-replay.mp4', writer=FFMpegWriter(fps=fps, codec='libx264',
                   extra_args=['-pix_fmt', 'yuv420p', '-movflags', '+faststart']), dpi=100)
    plt.close(fig)
    return cases


def render(full, dev16, output):
    comparison = compare(full, output)
    if not comparison['v1']['complete']:
        raise ValueError('full benchmark is incomplete')
    navigation_figure(full, output)
    mapping_figure(full, output)
    metrics_figure(comparison, output)
    cases = replay(dev16, output)
    sources = [case['path'] / name for case in cases for name in
               ('nav2-goal.json', 'navigation-trajectories.npz', 'run-provenance.json')]
    sources.extend(path for path in output.iterdir() if path.name != 'evidence.json')
    evidence = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (output / 'evidence.json').write_text(json.dumps(evidence, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--full', type=Path, required=True)
    parser.add_argument('--dev16', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=new_output('reports', 'evaluation'))
    args = parser.parse_args()
    render(args.full.resolve(), args.dev16.resolve(), args.output.resolve())
