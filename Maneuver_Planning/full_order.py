#  Full classical optimization
import numpy as np

from scipy.optimize import minimize
from config import (T_BURN, PLANNING_SEPARATION, MAX_DELTA_V)

from dynamics import trajectory_maps


def optimize_burn(times, A, x0):
    nominal, burn_map = trajectory_maps(times, A, x0, T_BURN)

    r_nominal = nominal[:, :3]
    G = burn_map[:, :3, :]

    # Optimize dimensionless q, where delta_v = MAX_DELTA_V * q.
    def positions(q):
        return (
            r_nominal
            + np.einsum("tij,j->ti", G, MAX_DELTA_V * q)
        )

    def objective(q):
        return 0.5 * np.dot(q, q)

    def clearance_constraint(q):
        r = positions(q)

        # Inequality constraints must be >= 0.
        return (
            np.sum(r * r, axis=1) / PLANNING_SEPARATION**2 - 1
        )

    def burn_limit_constraint(q):
        return 1 - np.dot(q, q)

    constraints = [
        {"type": "ineq", "fun": clearance_constraint},
        {"type": "ineq", "fun": burn_limit_constraint},
    ]

    # Nonzero initial guesses help explore avoidance directions.
    guesses = [np.zeros(3)]
    for axis in np.eye(3):
        guesses.append(0.5 * axis)
        guesses.append(-0.5 * axis)

    feasible_results = []
    for guess in guesses:
        result = minimize(
            objective,
            guess,
            jac=lambda q: q,
            method="SLSQP",
            bounds=[(-1, 1)] * 3,
            constraints=constraints,
            options={
                "maxiter": 500,
                "ftol": 1e-10,
                "disp": False,
            }
        )
        clearance_ok = np.min(clearance_constraint(result.x)) >= -1e-6
        burn_ok = burn_limit_constraint(result.x) >= -1e-6

        if clearance_ok and burn_ok:
            feasible_results.append(result)

    if not feasible_results:
        raise RuntimeError("No feasible solution found. Inspect the encounter, burn limit, burn time, and solver diagnostics.")

    best = min(feasible_results, key=lambda item: item.fun)

    return MAX_DELTA_V * best.x, best
