"""Summarize actual oracle attempts only; never manufacture baseline rows."""
import argparse
import csv
import json
from pathlib import Path


def summarize(source, destination):
    rows = []
    for path in sorted(source.glob('*/result.json')):
        r = json.loads(path.read_text())
        if r.get('mode') != 'PHASE_2_ORACLE_GEOMETRY_ONLY':
            raise ValueError(f'Unexpected experiment type in {path}')
        geometry = r.get('final_geometry_gt', {})
        row = dict(run=path.parent.name, seed=r['seed'], mode=r['mode'],
                   success=r['success'], sim_steps=r['sim_steps'],
                   simulation_seconds=r['simulation_seconds'],
                   lateral_error_gt=geometry.get('lateral_error_gt'),
                   insertion_error_gt=geometry.get('insertion_error_gt'),
                   eligible_for_strategy_benchmark=r['eligible_for_strategy_benchmark'],
                   error=r.get('error'))
        assert row['eligible_for_strategy_benchmark'] is False
        dt = json.loads((path.parent/'config.json').read_text())['dt']
        assert abs(row['sim_steps']*dt-row['simulation_seconds']) < 1e-8
        rows.append(row)
    if not rows:
        raise ValueError('No recorded oracle attempts found')
    destination.mkdir(parents=True, exist_ok=True)
    with (destination/'phase2_attempts.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (destination/'phase2_attempts.json').write_text(json.dumps(rows, indent=2))
    print(f'{len(rows)} recorded oracle attempts; zero visual baseline episodes')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summarize(args.source, args.output)
