# Select planners and run experiments
import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path 
from maneuver_planning.scenarios.leo_500km import (MEAN_MOTION, T_CA, T_BURN, T_END, REQUIRED_SEPARATION)

from maneuver_planning.dynamics import hcw_matrix, trajectory_maps, apply_burn
from maneuver_planning.encounters import state_at_encounter, initial_state
from maneuver_planning.validation import separation, check_dynamics
from maneuver_planning.visualization.plotting import plot_avoidance
from maneuver_planning.planners.full_order import optimize_burn


A = hcw_matrix(MEAN_MOTION)
x0 = initial_state(state_at_encounter, T_CA, A)


def main():
    check_dynamics(A, x0, state_at_encounter, T_CA, T_BURN)

    # Optimize with 10 second spacing.
    planning_times = np.linspace(0, T_END, 181)

    delta_v, result = optimize_burn(planning_times, A, x0)

    # Independently inspect on a finer, 1-second grid.
    validation_times = np.linspace(0, T_END, 1801)
    nominal, burn_map = trajectory_maps(validation_times, A, x0, T_BURN)

    maneuvered = apply_burn(nominal, burn_map, delta_v)

    nominal_distance = separation(nominal)
    maneuvered_distance = separation(maneuvered)

    i_nominal = np.argmin(nominal_distance)
    i_maneuvered = np.argmin(maneuvered_distance)

    print(f"Burn epoch: {T_BURN:.1f} s")
    print(f"Delta_v [radial, along-track, normal]: {delta_v} m/s")
    print(f"Delta-v magnitude: {np.linalg.norm(delta_v):.6f} m/s")
    print(f"Iterations for selected solve: {result.nit}")
    print(
        f"Nominal sampled minimum: "
        f"{nominal_distance[i_nominal]:.3f} m "
        f"at {validation_times[i_nominal]:.1f} s"
    )
    print(
        f"Maneuvered sampled minimum: "
        f"{maneuvered_distance[i_maneuvered]:.3f} m "
        f"at {validation_times[i_maneuvered]:.1f} s"
    )
    if maneuvered_distance[i_maneuvered] < REQUIRED_SEPARATION:
        raise RuntimeError("Fine-grid validation failed. Refine the planning grid and investigate before accepting this maneuver.")

    print("Fine-grid validation passed.")
    
    # Save a readable, full-precision results as the reproducible 500 km baseline.
    results_dir = Path(__file__).resolve().parents[1] / "results" / "leo_500km"
    results_dir.mkdir(parents=True, exist_ok=True)
    baseline_file = results_dir / "full_order_LEO_test.txt"

    if baseline_file.exists():
        print(f"Baseline already exists; not overwriting: {baseline_file}")

    else:
        lines = [
            "500 km baseline: full-order classical planner",
            f"Burn epoch [s]: {T_BURN:.1f}",
            f"Delta_v radial [m/s]: {delta_v[0]:.17g}",
            f"Delta_v along-track [m/s]: {delta_v[1]:.17g}",
            f"Delta_v normal [m/s]: {delta_v[2]:.17g}",
            f"Delta-v magnitude [m/s]: {np.linalg.norm(delta_v):.17g}",
            f"Iterations: {result.nit}",
            f"Nominal minimal separation [m]: {nominal_distance[i_nominal]:.17g}",
            f"Nominal minimal separation time [s]: {validation_times[i_nominal]:.17g}",
            f"Maneuvered minimal separation [m]: {maneuvered_distance[i_maneuvered]:.17g}",
            f"Maneuvered minimal separation time [s]: {validation_times[i_maneuvered]:.17g}",
        ]
        baseline_file.write_text("\n".join(lines) + "\n")
        print(f"Baseline saved to: {baseline_file}")

        np.savez(
            baseline_file.with_suffix(".npz"),
            delta_v=delta_v,
            nit=result.nit,
            validation_times=validation_times,
            nominal_distance=nominal_distance,
            maneuvered_distance=maneuvered_distance,
        )

    # Figures & visualizations
    plot_avoidance(validation_times, nominal, maneuvered, nominal_distance, maneuvered_distance, REQUIRED_SEPARATION, T_BURN)

    plt.show()


if __name__ == "__main__":
    main()
