#  Check separation and closest approach
import numpy as np
from maneuver_planning.dynamics import phi, B_IMPULSE


def separation(states):
    """Distance from the HCW origin (the threat sits at the reference point)."""
    return np.linalg.norm(states[:, :3], axis=1)


def check_dynamics(A, x0, state_at_encounter, t_ca, t_burn):
    # Zero elapsed time must leave the state unchanged.
    assert np.allclose(phi(0, A), np.eye(6))

    # Propagate backward and forward to recover the encounter state. 
    assert np.allclose(phi(t_ca, A) @ x0, state_at_encounter, atol=1e-8)

    # Two propagation steps must agree with one combined step.
    assert np.allclose(phi(200, A) @ phi(100, A), phi(300, A), atol=1e-9)

    # An impulse changes only velocity at instant of the burn. 
    dv_test = np.array([0.01, 0.02, -0.03])
    before = phi(t_burn, A) @ x0
    after = before + B_IMPULSE @ dv_test

    assert np.allclose(after[:3], before[:3])
    assert np.allclose(after[3:] - before[3:], dv_test)

    print("Dynamics checks passed.")
    