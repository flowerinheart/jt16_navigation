#!/usr/bin/env python3
from pathlib import Path
import subprocess,yaml,sys
root=Path(__file__).resolve().parents[1]; failures=[]
roles=sys.argv[1:]
manifests=[root/f'dependencies.{role}.repos' for role in roles] if roles else root.glob('dependencies.*.repos')
for manifest in manifests:
 for path,item in yaml.safe_load(manifest.read_text())['repositories'].items():
  p=root/path
  actual=subprocess.run(['git','-C',str(p),'rev-parse','HEAD'],capture_output=True,text=True)
  ok=actual.returncode==0 and actual.stdout.strip()==item['version']
  print(('PASS' if ok else 'FAIL'),path)
  if not ok:failures.append(path)
sys.exit(bool(failures))
