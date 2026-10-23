# Maneuver Planning

This repository contains a 500 km Low Earth Orbit (LEO) collision-avoidance maneuver-planning baseline. The experiment uses HCW relative dynamics and a full-order, single-impulse optimization with SciPy SLSQP.

## Reproduce the LEO baseline

### 1. Clone the repository

```bash
git clone https://github.com/tanerjn/mp.git
cd mp
```

If already cloned, run `git pull origin main` from the repository root.

### 2. Create and activate a virtual environment

Python 3.10 or newer is required.

**macOS / Linux:**

```bash
python3 -m venv .venv
source .venv/bin/activate
```

**Windows PowerShell:**

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

### 3. Install dependencies

Run from the repository root:

```bash
python -m pip install -e .
```

This installs NumPy, SciPy, and Matplotlib as declared in `pyproject.toml`.

### 4. Run the simulation

```bash
python -m examples.run_leo_baseline
```

The script sets up the orbital encounter, checks HCW dynamics, optimizes the avoidance impulse, validates separation on a finer time grid, and displays trajectory plots.

### 5. Inspect the output

A successful run prints `Dynamics checks passed.`, the optimized delta-v and minimum separations, and `Fine-grid validation passed.`.

Generated files are saved to:

```text
results/
└── leo_500km/
    ├── full_order_LEO_test.txt
    └── full_order_LEO_test.npz
```

- `full_order_LEO_test.txt`: human-readable maneuver and validation summary.
- `full_order_LEO_test.npz`: numerical arrays for subsequent analysis.

The program also displays separation-versus-time and relative-trajectory plots.

**Rerunning:** If `full_order_LEO_test.txt` already exists, the script does not overwrite either output. Move or delete both generated files if you need a fresh result.

### 6. Check success criteria

The maneuver must satisfy the required minimum separation of **100 m** and maximum impulse magnitude of **0.5 m/s**. The run must also pass the fine-grid validation check. A successful numerical run is not guaranteed for every modified scenario.

## Repository layout

- `maneuver_planning/`: reusable dynamics, encounter generation, validation, optimization, and LEO scenario configuration.
- `examples/run_leo_baseline.py`: runnable LEO baseline entry point.
- `results/leo_500km/`: baseline and generated outputs.
- `Maneuver_Planning/mp_project_review.pdf`: project review document.

The legacy `__Maneuver_Planning/` directory contains failed-export placeholders retained for reference; these are not executable code. GEO and reduced-order/hybrid implementations have not yet been restored.
