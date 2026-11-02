#!/usr/bin/env python3
"""Reproduce the same-PCD comparison; source scripts/env.sh first."""
import json
import subprocess
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent
EXE = ROOT/'install/robocon_cloud_localization/lib/robocon_cloud_localization/relocalization_benchmark'

def main():
    results = []
    for case, target, source, repeats in [
        ('synthetic', OUT/'synthetic_map.pcd', OUT/'synthetic_scan.pcd', 1),
        ('symmetric', OUT/'symmetric_map.pcd', OUT/'synthetic_scan.pcd', 1),
        ('simulation', ROOT/'src/robocon_cloud_localization/maps/mid360_sim_2.pcd', OUT/'simulation_scan.pcd', 3),
    ]:
        for backend in ('grid_gicp', 'bbs_cpu_gicp', 'bbs_gpu_gicp'):
            run = subprocess.run([str(EXE), str(target), str(source), backend,
                                  str(repeats), '0', '0', '1.5708'],
                                 capture_output=True, text=True, timeout=60)
            (OUT/f'{case}_{backend}.log').write_text(run.stdout+'\n'+run.stderr)
            rows = [json.loads(line[len('RESULT '):]) for line in run.stdout.splitlines()
                    if line.startswith('RESULT ')]
            assert len(rows) == repeats, run.stderr
            assert run.returncode == (1 if case == 'symmetric' else 0), run.stderr
            if case == 'symmetric':
                assert 'ambiguous global matches' in run.stderr, run.stderr
            for row in rows:
                assert row['found'] == (case != 'symmetric'), row
                row['case'] = case
                results.append(row)
                print(case, backend, row['run'], row['found'], row['total_ms'], flush=True)
    (OUT/'comparison.json').write_text(json.dumps(results, indent=2)+'\n')

if __name__ == '__main__':
    main()
