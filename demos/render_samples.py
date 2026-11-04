"""Render selected recorded full-evaluation cases; never run simulation."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'docs/v1'))
from render_report import load_navigation, map_panel, local_trajectory
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, Polygon
import numpy as np

OUT = ROOT / 'demos'
RUNS = {
    'v0': ROOT / 'integration/results/v0/full-benchmark-20261101',
    'v1-candidate-g': ROOT / 'integration/results/v1/full-benchmark-20261104-g',
}
CASES = {
    'v0': [('success', 'arena6/small_warehouse_obs05/seed_00'),
           ('failure', 'arena6/small_warehouse_obs10/seed_00'),
           ('failure', 'arena6/map2_obs10/seed_00')],
    'v1-candidate-g': [('success', 'barn30/world_022/navigation/seed_00'),
                       ('failure', 'barn30/world_111/navigation/seed_03'),
                       ('failure', 'barn30/world_266/navigation/seed_00')],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(version, source, output, metrics, files):
    assert output.stat().st_size > 0
    return dict(version=version, source=str(source.relative_to(ROOT)),
                output=str(output.relative_to(OUT)), metrics=metrics,
                source_sha256={str(p.relative_to(ROOT)): digest(p) for p in files},
                output_sha256=digest(output))


def mapping(version, world):
    source = RUNS[version] / f'barn30/world_{world:03d}/mapping'
    result = json.loads((source / 'mapping-result.json').read_text())
    metrics = json.loads((source / 'map-metrics.json').read_text())
    manifest = json.loads((ROOT / 'integration/benchmarks/manifests/barn30.json').read_text())
    entry = next(e for e in manifest['entries'] if e['world_id'] == world)
    x, y, yaw = entry['start_xyyaw_m_rad']
    origin = SimpleNamespace(initial_x=x, initial_y=y, initial_yaw=yaw,
                             ground_truth_origin_x=None, ground_truth_origin_y=None,
                             ground_truth_origin_yaw=None)
    with np.load(source / 'mapping-trajectories.npz') as data:
        truth = np.asarray(local_trajectory(data['gt'][:, [1, 2, 3, 0]].tolist(), origin))
        lio = data['lio'].copy()
    assert np.isfinite(truth).all() and np.isfinite(lio).all()
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    map_yaml = source / 'nav-map/map.yaml'
    map_panel(ax, dict(gt=truth, identity={'parameters': {'map_yaml': str(map_yaml)}}))
    ax.plot(truth[:, 0], truth[:, 1], label='Acquisition path (GT)', color='#007f86')
    ax.plot(lio[:, 1], lio[:, 2], label='FAST-LIO', color='#d46a36')
    ax.legend()
    ax.set_title(f"{version} / full evaluation / BARN {world}\n"
                 f"ATE {result['ate_rmse_m']:.3f} m | occupied F1 {metrics['occupied_f1']:.3f}")
    output = OUT / version / 'mapping' / f'barn-{world:03d}.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    files = [source / f for f in ('mapping-result.json', 'map-metrics.json',
             'mapping-trajectories.npz', 'run-provenance.json', 'nav-map/map.yaml', 'nav-map/map.pgm')]
    return record(version, source, output, dict(ate_rmse_m=result['ate_rmse_m'],
                  occupied_f1=metrics['occupied_f1']), files)


def navigation(version, bucket, relative):
    case = load_navigation(RUNS[version], relative, relative)
    assert case is not None
    result, params = case['summary'], case['identity']['parameters']
    assert bool(result['scored_success']) == (bucket == 'success')
    gt = case['gt']
    goal = np.array(result['gt_goal_xy_m'])
    if params['barn_world']:
        gt = np.asarray(local_trajectory(gt.tolist(), SimpleNamespace(**params)))
        goal = np.asarray(local_trajectory([[*goal, 0., 0.]], SimpleNamespace(**params)))[0, :2]
    case['gt'] = gt
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    map_panel(ax, case)
    ax.update_datalim([goal])
    low, high = np.minimum(gt[:, :2].min(axis=0), goal)-1.2, np.maximum(gt[:, :2].max(axis=0), goal)+1.2
    ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]))
    ax.scatter(*goal, marker='*', s=120, color='#204c80', label='Goal')
    line, = ax.plot([], [], color='#007f86', label='Actual trajectory (GT)')
    robot = Polygon(case['footprint'], color='#007f86')
    ax.add_patch(robot)
    obstacles = []
    for agent in np.unique(case['dynamic'][:, 1]):
        samples = case['dynamic'][case['dynamic'][:, 1] == agent]
        circle = Circle(samples[0, 2:4], samples[0, 4], color='#d46a36', alpha=.75)
        ax.add_patch(circle)
        obstacles.append((circle, samples))
    ax.legend(loc='best', fontsize=8)
    status = 'STRICT PASS' if result['scored_success'] else 'STRICT FAIL'
    fig.suptitle(f'{version} / full evaluation / telemetry replay / 8x', fontsize=12)
    output = OUT / version / 'navigation' / bucket / (relative.replace('/', '-') + '.mp4')
    output.parent.mkdir(parents=True, exist_ok=True)
    fps, speed, duration = 12, 8, result['elapsed_s']
    writer = FFMpegWriter(fps=fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-movflags', '+faststart'])
    with writer.saving(fig, str(output), dpi=100):
        for index in range(math.ceil(duration * fps / speed) + 1):
            elapsed = min(index * speed / fps, duration)
            stamp = result['mission_start_stamp_s'] + elapsed
            row = np.clip(np.searchsorted(gt[:, 3], stamp, side='right') - 1, 0, len(gt)-1)
            x, y, yaw = gt[row, :3]
            c, s = math.cos(yaw), math.sin(yaw)
            robot.set_xy(case['footprint'] @ np.array([[c, s], [-s, c]]) + [x, y])
            line.set_data(gt[:row+1, 0], gt[:row+1, 1])
            for circle, samples in obstacles:
                observation = samples[np.abs(samples[:, 0] - stamp).argmin()]
                circle.set_visible(abs(observation[0] - stamp) <= .25)
                circle.center, circle.radius = tuple(observation[2:4]), observation[4]
            ax.set_title(f"{relative}\n{status} | action {result['result']} | collision {result['collision']}\n"
                         f"t={elapsed:.1f}/{duration:.1f}s | final goal distance {result['final_goal_distance_m']:.3f}m")
            writer.grab_frame()
    fig.savefig(output.with_suffix('.png'), dpi=100)
    plt.close(fig)
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-show_entries',
        'stream=codec_name,width,height:format=duration,size', '-of', 'json', str(output)]))
    assert probe['streams'][0]['codec_name'] == 'h264'
    assert abs(float(probe['format']['duration']) - (math.ceil(duration*fps/speed)+1)/fps) < .1
    files = [case['path']/f for f in ('nav2-goal.json', 'navigation-trajectories.npz', 'run-provenance.json', 'nav2-params.yaml')]
    files.append(Path(params['map_yaml']))
    item = record(version, case['path'], output, {k:result[k] for k in
        ('result', 'scored_success', 'collision', 'elapsed_s', 'final_goal_distance_m', 'input_failure')}, files)
    item.update(video=probe, poster=str(output.with_suffix('.png').relative_to(OUT)),
                poster_sha256=digest(output.with_suffix('.png')))
    return item


if __name__ == '__main__':
    records = []
    for version in RUNS:
        for world in (22, 144, 255):
            records.append(mapping(version, world))
        for bucket, relative in CASES[version]:
            records.append(navigation(version, bucket, relative))
            print(f'Rendered {version} {relative}', flush=True)
    assert len(records) == 12
    (OUT / 'samples.json').write_text(json.dumps(records, indent=2)+'\n')
