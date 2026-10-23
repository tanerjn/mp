#  HCW matrix and state propagation
import numpy as np

from scipy.linalg import expm

# HCW Matrix
def hcw_matrix(n):
    """State order: [rx, ry, rz, vx, vy, vz]."""
    return np.array([
                  [0,     0,   0,   1,  0,  0],
                  [0,     0,   0,   0,  1,  0],
                  [0,     0,   0,   0,  0,  1],
                  [3*n*n, 0,   0,   0, 2*n, 0],
                  [0,     0,   0, -2*n, 0,  0],
                  [0,     0, -n*n,  0,  0,  0],
    ], dtype=float)

# An impulse burn changes velocity but not position.
B_IMPULSE = np.vstack([
    np.zeros((3, 3)),
    np.eye(3)
])

# Phi
def phi(t, A):
    """HCW state-transition matrix for time t, given system matrix A"""
    return expm(A * t)


def trajectory_maps(times, A, x0,t_burn):
    """Return maps satisfying:
    state(t) = nominal state(t) + burn_map(t) @ delta_v"""
    nominal = np.array([phi(t, A) @ x0 for t in times])
    burn_map = np.zeros((len(times), 6, 3))
    
    for i, t in enumerate(times):
        if t >= t_burn: 
            burn_map[i] = phi(t - t_burn, A) @ B_IMPULSE

    return nominal, burn_map


def apply_burn(nominal, burn_map, delta_v):
    return nominal + np.einsum("tij,j->ti", burn_map, delta_v)
