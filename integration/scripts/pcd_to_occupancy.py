#!/usr/bin/env python3
"""Deterministically rasterize a binary FAST-LIO PCD into a Nav2 map."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from benchmark_geometry import dilate_occupied


parser = argparse.ArgumentParser()
parser.add_argument('--pcd', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--resolution', type=float, default=.05)
parser.add_argument('--min-height', type=float, default=.05)
parser.add_argument('--max-height', type=float, default=.60)
parser.add_argument('--padding', type=float, default=.5)
parser.add_argument('--trajectory', type=Path)
parser.add_argument('--trajectory-padding', type=float, default=8.0)
parser.add_argument('--max-trajectory-radius', type=float, default=12.0)
parser.add_argument('--min-hits', type=int, default=50)
args = parser.parse_args()
if args.resolution<=0 or args.min_hits<1 or args.min_height>=args.max_height:
    raise ValueError('invalid rasterization parameters')

with args.pcd.open('rb') as stream:
    header = []
    while True:
        line = stream.readline()
        if not line:
            raise RuntimeError('PCD header ended before DATA')
        decoded = line.decode('ascii').strip()
        header.append(decoded)
        if decoded.startswith('DATA '):
            break
    payload = stream.read()

meta = {line.split()[0]: line.split()[1:] for line in header if line and not line.startswith('#')}
if meta['DATA'][0] != 'binary':
    raise RuntimeError('only binary PCD is supported')
fields = meta['FIELDS']
sizes = list(map(int, meta['SIZE']))
types = meta['TYPE']
counts = list(map(int, meta.get('COUNT', ['1'] * len(fields))))
formats = []
for size, kind, count in zip(sizes, types, counts):
    base = {
        ('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2',
        ('U', 4): '<u4', ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4'
    }[(kind, size)]
    formats.append((base, (count,)) if count > 1 else base)
dtype = np.dtype({'names': fields, 'formats': formats})
point_count = int(meta['POINTS'][0])
cloud = np.frombuffer(payload, dtype=dtype, count=point_count)
xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(float)
finite = np.isfinite(xyz).all(axis=1)
# FAST-LIO's initialized world origin is not guaranteed to lie on the floor.
# Estimate the dominant horizontal surface, then apply robot-relative heights.
z_values = xyz[finite, 2]
histogram, edges = np.histogram(z_values, bins=max(64, int(np.sqrt(len(z_values)))))
floor_z = float((edges[np.argmax(histogram)] + edges[np.argmax(histogram) + 1]) * .5)
valid = finite & (xyz[:, 2] >= floor_z + args.min_height) & (xyz[:, 2] <= floor_z + args.max_height)
points = xyz[valid]
if not len(points):
    raise RuntimeError('no points remain after height filtering')

trajectory = np.load(args.trajectory)['lio'] if args.trajectory else None
if trajectory is not None:
    if trajectory.ndim != 2 or trajectory.shape[1] != 4 or not len(trajectory) or not np.isfinite(trajectory).all():
        raise ValueError('invalid mapping trajectory')
    poses = trajectory[:, 1:3]
    bounded = poses[np.linalg.norm(poses - poses[0], axis=1) <= args.max_trajectory_radius]
    minimum = bounded.min(axis=0) - args.trajectory_padding
    maximum = bounded.max(axis=0) + args.trajectory_padding
    inside = ((points[:, :2] >= minimum) & (points[:, :2] <= maximum)).all(axis=1)
    points = points[inside]
else:
    minimum = points[:, :2].min(axis=0) - args.padding
    maximum = points[:, :2].max(axis=0) + args.padding
size = np.ceil((maximum - minimum) / args.resolution).astype(int) + 1
# Aggregated PCDs retain obstacle surfaces but not per-scan origins needed for
# ray tracing. Keep unobserved cells unknown rather than inventing free space.
grid = np.full((size[1], size[0]), 205, dtype=np.uint8)
cells = np.floor((points[:, :2] - minimum) / args.resolution).astype(int)
hit_counts = np.zeros_like(grid, dtype=np.uint32)
np.add.at(hit_counts, (cells[:, 1], cells[:, 0]), 1)
grid[hit_counts >= args.min_hits] = 0
grid[dilate_occupied(grid == 0)] = 0

# Traversed footprint can label unknown cells free, never erase occupied returns.
if trajectory is not None:
    for _, x, y, yaw in trajectory:
        center = np.floor((np.asarray([x, y]) - minimum) / args.resolution).astype(int)
        radius = int(np.ceil(.31 / args.resolution))
        x0, x1 = max(0, center[0] - radius), min(grid.shape[1], center[0] + radius + 1)
        y0, y1 = max(0, center[1] - radius), min(grid.shape[0], center[1] + radius + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        yy, xx = np.mgrid[y0:y1, x0:x1]
        dx = minimum[0] + (xx + .5) * args.resolution - x
        dy = minimum[1] + (yy + .5) * args.resolution - y
        bx = np.cos(yaw) * dx + np.sin(yaw) * dy
        by = -np.sin(yaw) * dx + np.cos(yaw) * dy
        region = grid[y0:y1, x0:x1]
        region[(np.abs(bx) <= .20) & (np.abs(by) <= .235) & (region == 205)] = 254

args.output.mkdir(parents=True, exist_ok=True)
Image.fromarray(np.flipud(grid), mode='L').save(args.output / 'map.pgm')
yaml = (
    f"image: map.pgm\nmode: trinary\nresolution: {args.resolution}\n"
    f"origin: [{minimum[0]}, {minimum[1]}, 0.0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
)
(args.output / 'map.yaml').write_text(yaml)
summary = {
    'source': str(args.pcd), 'input_points': point_count,
    'height_filtered_points': int(len(points)), 'resolution_m': args.resolution,
    'estimated_floor_z_m': floor_z,
    'height_range_above_floor_m': [args.min_height, args.max_height],
    'origin_xy_m': minimum.tolist(), 'size_cells': size.tolist(),
    'occupied_cells': int((grid == 0).sum()), 'bounds_xy_m': [minimum.tolist(), maximum.tolist()]
    , 'trajectory_cleared_occupied_cells': 0, 'minimum_hits_per_cell': args.min_hits
}
(args.output / 'map-summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
