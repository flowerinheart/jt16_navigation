#!/usr/bin/env python3
"""Score candidate base_link height bands for a 2D scan projection."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()

points = np.load(args.input / "lidar_base_xyz.npy")
points = points[np.isfinite(points).all(axis=1)]
xy_range = np.linalg.norm(points[:, :2], axis=1)
angles = np.mod(np.arctan2(points[:, 1], points[:, 0]), 2 * np.pi)

# Geometry is defined by build_world.py and is used only for offline diagnosis.
is_ground = points[:, 2] <= 0.05
is_self = (
    (points[:, 0] >= -0.23) & (points[:, 0] <= 0.23)
    & (points[:, 1] >= -0.26) & (points[:, 1] <= 0.26)
    & (points[:, 2] <= 0.75)
)
is_low_box = (
    (points[:, 0] >= 1.28) & (points[:, 0] <= 1.72)
    & (np.abs(points[:, 1]) <= 0.22)
    & (points[:, 2] >= -0.02) & (points[:, 2] <= 0.32)
)
is_wall = (
    ((points[:, 0] >= 3.88) & (points[:, 0] <= 4.12) & (np.abs(points[:, 1]) <= 4.1))
    | ((points[:, 1] >= 3.88) & (points[:, 1] <= 4.12) & (np.abs(points[:, 0]) <= 4.1))
)

bands = [
    (0.05, 0.25),
    (0.10, 0.35),
    (0.15, 0.50),
    (0.20, 0.60),
    (0.25, 1.00),
]
bin_count = 300
bin_index = np.floor(angles / (2 * np.pi) * bin_count).astype(int) % bin_count

rows = []
for min_height, max_height in bands:
    eligible = (
        (points[:, 2] >= min_height)
        & (points[:, 2] <= max_height)
        & (xy_range >= 0.15)
        & (xy_range <= 30.0)
    )
    def nearest_by_bin(mask):
        result = []
        for index in range(bin_count):
            candidates = np.flatnonzero(mask & (bin_index == index))
            if candidates.size:
                result.append(int(candidates[np.argmin(xy_range[candidates])]))
        return np.asarray(result, dtype=int)

    selected = nearest_by_bin(eligible)
    selected_without_self = nearest_by_bin(eligible & ~is_self)
    count = int(selected.size)
    filtered_count = int(selected_without_self.size)
    rows.append({
        "min_height_m": min_height,
        "max_height_m": max_height,
        "eligible_points": int(eligible.sum()),
        "scan_bins_with_return": count,
        "scan_coverage_pct": 100.0 * count / bin_count,
        "selected_ground_false_returns": int(is_ground[selected].sum()) if count else 0,
        "selected_self_false_returns": int(is_self[selected].sum()) if count else 0,
        "selected_low_box_returns": int(is_low_box[selected].sum()) if count else 0,
        "selected_wall_returns": int(is_wall[selected].sum()) if count else 0,
        "nearest_selected_range_m": float(xy_range[selected].min()) if count else None,
        "median_selected_range_m": float(np.median(xy_range[selected])) if count else None,
        "after_self_filter_scan_bins": filtered_count,
        "after_self_filter_coverage_pct": 100.0 * filtered_count / bin_count,
        "after_self_filter_low_box_returns": int(is_low_box[selected_without_self].sum()) if filtered_count else 0,
        "after_self_filter_wall_returns": int(is_wall[selected_without_self].sum()) if filtered_count else 0,
        "after_self_filter_nearest_range_m": float(xy_range[selected_without_self].min()) if filtered_count else None,
    })

summary = {
    "source": str(args.input),
    "scope": "One static ideal Gazebo scan; offline diagnosis, not dynamic-navigation acceptance.",
    "mount": {"xyz_m": [-0.03, 0.0, 0.36], "pitch_up_rad": json.loads((args.input / "summary.json").read_text()).get("mount_pitch_up_rad", 0.676350)},
    "world_geometry": {
        "low_box": {"center_m": [1.5, 0.0, 0.15], "size_m": [0.4, 0.4, 0.3]},
        "front_wall_x_m": 4.0,
        "left_wall_y_m": 4.0,
    },
    "input": {
        "valid_points": int(points.shape[0]),
        "z_min_m": float(points[:, 2].min()),
        "z_max_m": float(points[:, 2].max()),
        "points_below_0_05m": int(is_ground.sum()),
        "points_within_0_5m": int((xy_range < 0.5).sum()),
        "points_in_robot_footprint_mask": int(is_self.sum()),
        "low_box_points_all_heights": int(is_low_box.sum()),
        "wall_points_all_heights": int(is_wall.sum()),
    },
    "bands": rows,
}

args.output.mkdir(parents=True, exist_ok=True)
(args.output / "projection_analysis.json").write_text(json.dumps(summary, indent=2))
with (args.output / "projection_bands.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

print(json.dumps(summary, indent=2))
