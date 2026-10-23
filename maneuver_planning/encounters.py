#  Generate conjunction initial conditions
import numpy as np

from maneuver_planning.dynamics import phi

# At T_CA: 10 m radial separation, 1 m/s along-track speed.
state_at_encounter = np.array([
    10, 0, 0,
     0, 1, 0,
])


def initial_state(state_at_encounter, t_ca, A):
    """Propagate the encounter state back from t_ca to t = 0."""
    return phi(-t_ca, A) @ state_at_encounter
