# Maneuver Planning

Orbital collision-avoidance maneuver planning with a 500 km LEO single-impulse baseline.

## Run

From the repository root:

```bash
python -m pip install -e .
python -m examples.run_leo_baseline
```

The run checks HCW dynamics, optimizes a single impulse with SciPy SLSQP, validates separation on a finer grid, writes to `results/leo_500km/`, and displays plots.

## Layout

- `maneuver_planning/`: reusable code and LEO scenario settings.
- `examples/`: runnable scripts.
- `results/leo_500km/`: baseline data and generated output.
- `Maneuver_Planning/mp_project_review.pdf`: project review document (unchanged).

The legacy `__Maneuver_Planning/` contains failed-export placeholders, retained for reference. GEO and reduced-order/hybrid planner implementations have not been restored.
