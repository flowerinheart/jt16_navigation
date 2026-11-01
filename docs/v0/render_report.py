"""Render V0 evidence, not simulated camera footage. Run from any directory."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import markdown
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Circle, Polygon
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / 'assets'
DATA = ROOT / 'integration/results/v0/timed-mppi-startup-pilot'
sys.path.insert(0, str(ROOT / 'integration/scripts'))
from navigation_metrics import local_trajectory
from run_provenance import verify_provenance

plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                     'axes.spines.right': False, 'figure.facecolor': 'white'})
CASES = [('Warehouse', 'arena/small_warehouse_obs05/seed_01'),
         ('Map2', 'arena/map2_obs05/seed_00'), ('Map5', 'arena/map5_obs05/seed_01'),
         ('BARN22', 'barn/world_022/navigation/seed_00'),
         ('BARN55', 'barn/world_055/navigation/seed_00'),
         ('BARN77', 'barn/world_077/navigation/seed_00')]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_case(name, relative):
    path = DATA / relative
    verify_provenance(path)
    summary = json.loads((path / 'nav2-goal.json').read_text())
    assert summary['scored_success'] and not summary['collision']
    assert summary['localization_success'] and summary['input_failure'] is None
    identity = json.loads((path / 'run-provenance.json').read_text())
    archive = np.load(path / 'navigation-trajectories.npz')
    gt = archive['gt']
    gt = gt[(gt[:, 3] >= summary['mission_start_stamp_s']) &
            (gt[:, 3] <= summary['mission_end_stamp_s'])]
    if relative.startswith('barn/'):
        gt = np.asarray(local_trajectory(gt.tolist(), SimpleNamespace(**identity['parameters'])))
    return dict(name=name, path=path, summary=summary, identity=identity,
                gt=gt, amcl=archive['amcl'], dynamic=archive['dynamic'])


def map_panel(ax, case):
    path = Path(case['identity']['parameters']['map_yaml'])
    info = yaml.safe_load(path.read_text())
    pixels = np.asarray(Image.open(path.parent / info['image']))
    x, y = info['origin'][:2]
    res = info['resolution']
    ax.imshow(pixels, cmap='gray', vmin=0, vmax=255, origin='upper',
              extent=(x, x + pixels.shape[1] * res, y, y + pixels.shape[0] * res))
    xy = case['gt'][:, :2]
    low, high = xy.min(axis=0) - 1.2, xy.max(axis=0) + 1.2
    ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]),
           xlabel='x (m)', ylabel='y (m)')
    ax.set_aspect('equal', adjustable='box')
    return pixels


def trajectories(cases):
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), height_ratios=[3, 1], constrained_layout=True)
    for ax, case in zip(axes.flat, cases):
        map_panel(ax, case)
        gt, amcl = case['gt'], case['amcl']
        ax.plot(gt[:, 0], gt[:, 1], color='#007f86', lw=2.4, label='Actual trajectory (GT)')
        ax.plot(amcl[:, 0], amcl[:, 1], color='#e09228', lw=1.2, alpha=.85, label='AMCL estimate')
        ax.scatter(*gt[0, :2], color='#007f86', s=35, marker='o')
        ax.scatter(*gt[-1, :2], color='#007f86', s=70, marker='*')
        j = case['summary']
        ax.set_title(f"{case['name']}  /  PASS\n{j['elapsed_s']:.1f}s | min gap {j['minimum_obstacle_clearance_m']:.3f}m")
    axes[0, 0].legend(loc='best', fontsize=8)
    fig.savefig(OUT / 'navigation.png', dpi=160)
    plt.close(fig)


def metrics(cases):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
    names = [c['name'] for c in cases]
    times = [c['summary']['elapsed_s'] for c in cases]
    gaps = [c['summary']['minimum_obstacle_clearance_m'] for c in cases]
    for ax, values, title in zip(axes, [times, gaps], ['Completion time (s)', 'Minimum obstacle clearance (m)']):
        bars = ax.barh(names, values, color=['#d46a36' if n == 'BARN22' else '#007f86' for n in names])
        ax.invert_yaxis()
        ax.set_title(title)
        ax.bar_label(bars, fmt='%.3f', padding=4)
        ax.set_xlim(0, max(values) * 1.18)
    axes[0].axvline(150, color='#d46a36', ls='--', lw=1)
    axes[0].text(150, 5.45, 'NAV150', ha='center', color='#d46a36')
    fig.savefig(OUT / 'metrics.png', dpi=160)
    plt.close(fig)


def mapping(cases):
    fig, axes = plt.subplots(1, 3, figsize=(12, 2.3), constrained_layout=True)
    for ax, case in zip(axes, cases[3:]):
        map_panel(ax, case)
        path = case['path'].parents[1] / 'mapping'
        archive = np.load(path / 'mapping-trajectories.npz')
        truth = archive['gt'][:, [1, 2, 3, 0]]
        truth = np.asarray(local_trajectory(truth.tolist(), SimpleNamespace(**case['identity']['parameters'])))
        lio = archive['lio']
        ax.plot(truth[:, 0], truth[:, 1], color='#007f86', lw=2, label='Acquisition path (GT)')
        ax.plot(lio[:, 1], lio[:, 2], color='#e09228', lw=1, label='FAST-LIO estimate')
        j = json.loads((path / 'mapping-result.json').read_text())
        m = json.loads((path / 'map-metrics.json').read_text())
        ax.set_title(f"{case['name']} / {j['waypoints_reached']}/{j['waypoints_total']}\nATE {j['ate_rmse_m']:.3f}m | F1 {m['occupied_f1']:.3f}")
    axes[0].legend(fontsize=8)
    fig.savefig(OUT / 'mapping.png', dpi=160)
    plt.close(fig)


def replay(cases):
    fig, axes = plt.subplots(1, 3, figsize=(12, 5), constrained_layout=True)
    artists = []
    for ax, case in zip(axes, cases[:3]):
        map_panel(ax, case)
        line, = ax.plot([], [], color='#007f86', lw=2)
        robot = Polygon(np.zeros((4, 2)), color='#007f86')
        ax.add_patch(robot)
        obstacles = [Circle((0, 0), .3, color='#d46a36', alpha=.75) for _ in range(5)]
        for obstacle in obstacles:
            ax.add_patch(obstacle)
        artists.append((line, robot, obstacles))
    duration = max(c['summary']['elapsed_s'] for c in cases[:3])
    fps, speed = 12, 8

    def frame(index):
        elapsed = min(index * speed / fps, duration)
        for ax, case, (line, robot, obstacles) in zip(axes, cases[:3], artists):
            stamp = case['summary']['mission_start_stamp_s'] + min(elapsed, case['summary']['elapsed_s'])
            gt = case['gt']
            row = int(np.searchsorted(gt[:, 3], stamp, side='right')) - 1
            row = max(0, min(row, len(gt) - 1))
            x, y, yaw = gt[row, :3]
            c, s = math.cos(yaw), math.sin(yaw)
            corners = np.array([[-.20, -.235], [.20, -.235], [.20, .235], [-.20, .235]])
            robot.set_xy(corners @ np.array([[c, s], [-s, c]]) + [x, y])
            line.set_data(gt[:row + 1, 0], gt[:row + 1, 1])
            for agent, obstacle in enumerate(obstacles):
                samples = case['dynamic'][case['dynamic'][:, 1] == agent]
                nearest = samples[np.argmin(np.abs(samples[:, 0] - stamp))]
                assert abs(nearest[0] - stamp) <= .25
                obstacle.center = tuple(nearest[2:4])
            ax.set_title(f"{case['name']} / t={min(elapsed, case['summary']['elapsed_s']):.1f}s")
        fig.suptitle('V0 Arena / telemetry replay, 8x / NOT camera footage', fontsize=12)

    video = FuncAnimation(fig, frame, frames=math.ceil(duration * fps / speed) + 1)
    frame(0)
    fig.savefig(OUT / 'replay-poster.png', dpi=120)
    video.save(OUT / 'arena-replay.mp4', writer=FFMpegWriter(fps=fps, codec='libx264',
               extra_args=['-pix_fmt', 'yuv420p', '-movflags', '+faststart']), dpi=100)
    plt.close(fig)


def evidence(cases):
    records = []
    for case in cases:
        path = case['path']
        files = ['nav2-goal.json', 'run-provenance.json', 'navigation-trajectories.npz',
                 'navigation-sensors.npz', 'tf-audit.json']
        records.append({'case': case['name'], 'run': str(path.relative_to(ROOT)),
                        'metrics': case['summary'], 'sha256': {f: digest(path / f) for f in files}})
    mapping_records = []
    for case in cases[3:]:
        path = case['path'].parents[1] / 'mapping'
        verify_provenance(path)
        identity = json.loads((path / 'run-provenance.json').read_text())
        assert identity['sources'] == cases[0]['identity']['sources']
        assert identity['runtime_hashes'] == cases[0]['identity']['runtime_hashes']
        mapping_records.append({'run': str(path.relative_to(ROOT)),
            'metrics': json.loads((path / 'mapping-result.json').read_text()),
            'map_metrics': json.loads((path / 'map-metrics.json').read_text()),
            'bundle': json.loads((path / 'map-bundle-metadata.json').read_text())})
    payload = {'version': 'v0', 'code_commit': subprocess.check_output(
        ['git', 'log', '-1', '--format=%H', 'v0', '--', 'VERSION'], cwd=ROOT, text=True).strip(),
        'scope': 'six-case small-simulation baseline; not full or hardware acceptance',
        'sources': cases[0]['identity']['sources'],
        'runtime_hashes': cases[0]['identity']['runtime_hashes'],
        'navigation': records, 'mapping': mapping_records,
        'excluded_failure': 'integration/results/v0/timed-mppi-current-gate/arena/map2_obs05/seed_00/nav2-goal.json',
        'media': {p.name: digest(p) for p in OUT.iterdir() if p.is_file()}}
    (OUT.parent / 'evidence.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')


def html_report():
    report = OUT.parent / 'report.md'
    body = markdown.markdown(report.read_text(), extensions=['tables', 'fenced_code'])
    style = """body{max-width:1080px;margin:40px auto;padding:0 24px;font:16px/1.7 system-ui,sans-serif;color:#18383c;background:#fff}
    h1{font-size:38px;margin-bottom:8px}h2{font-size:24px;margin-top:42px;border-bottom:1px solid #dce8e8;padding-bottom:8px}
    a{color:#007f86}img{max-width:100%;height:auto}table{border-collapse:collapse;width:100%;margin:20px 0;font-size:14px}
    td,th{padding:10px 12px;border-bottom:1px solid #dce8e8;text-align:left}th{background:#edf6f5}
    code{background:#edf3f3;padding:2px 5px;border-radius:4px;font-size:13px}details{padding:12px;background:#f5f8f8;border-radius:8px}
    @media(max-width:700px){body{padding:0 12px;margin:20px auto}td,th{padding:8px 5px;font-size:12px}h1{font-size:30px}}
    @media print{details{display:none}video{display:none}h2{break-after:avoid}img{max-height:520px;object-fit:contain}}"""
    (OUT.parent / 'report.html').write_text(
        '<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1">'
        '<title>V0 技术报告</title><style>' + style + '</style><body>' + body + '</body></html>')


if __name__ == '__main__':
    cases = [load_case(*case) for case in CASES]
    for case in cases:
        assert case['identity']['sources'] == cases[0]['identity']['sources']
        assert case['identity']['runtime_hashes'] == cases[0]['identity']['runtime_hashes']
    OUT.mkdir(exist_ok=True)
    trajectories(cases)
    metrics(cases)
    mapping(cases)
    replay(cases)
    evidence(cases)
    html_report()
    print(f'PASS: six source-verified cases rendered to {OUT}')
