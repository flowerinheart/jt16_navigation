#!/usr/bin/env python3
"""Audit actual TF publisher identities, including transient-local static edges."""
import argparse
import json
import subprocess
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from benchmark_geometry import tf_valid

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--duration', type=float, default=15.)
args = parser.parse_args()
executable = Path(get_package_prefix('jt16_bringup'))/'lib/jt16_bringup/tf_publisher_probe'
process = subprocess.run([str(executable), '--ros-args', '-p', f'duration_s:={args.duration}'],
                         capture_output=True, text=True, timeout=args.duration+10)
process.check_returncode()
report = json.loads(process.stdout)
report['duplicate_edges'] = {edge: owners for edge, owners in report['edges'].items() if len(owners)>1}
report['observed_topics'] = ['/tf', '/tf_static']
report['pass'] = tf_valid(report['edges'])
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['pass'] else 1)
