"""
Cinematica diretta e Jacobiano del Franka Research 3 (solo numpy).

Serve al controllo pre-esecuzione delle traiettorie (vedi
trajectory_validation.py) per stimare la vicinanza a una singolarità senza
dover interrogare il robot o MoveIt. Convenzione Craig (modified DH) come da
documentazione Franka; il punto di riferimento è la flangia (d_flange = 0.107 m).
"""

import numpy as np

# (a_{i-1}, d_i, alpha_{i-1}) per i giunti 1..7
_DH = (
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -np.pi / 2),
    (0.0, 0.316, np.pi / 2),
    (0.0825, 0.0, np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0, 0.0, np.pi / 2),
    (0.088, 0.0, np.pi / 2),
)
FLANGE_OFFSET = 0.107

FR3_Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
FR3_Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
FR3_DQ_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


def _dh_transform(theta: np.ndarray, a: float, d: float, alpha: float) -> np.ndarray:
    """Trasformazione T_{i-1,i} per un batch di angoli theta (N,)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.zeros((theta.shape[0], 4, 4))
    T[:, 0, 0] = ct
    T[:, 0, 1] = -st
    T[:, 0, 3] = a
    T[:, 1, 0] = st * ca
    T[:, 1, 1] = ct * ca
    T[:, 1, 2] = -sa
    T[:, 1, 3] = -sa * d
    T[:, 2, 0] = st * sa
    T[:, 2, 1] = ct * sa
    T[:, 2, 2] = ca
    T[:, 2, 3] = ca * d
    T[:, 3, 3] = 1.0
    return T


def forward_kinematics_and_jacobian(q: np.ndarray):
    """
    Calcola posa della flangia e Jacobiano geometrico (6x7, nel frame base).

    Args:
        q: array (7,) oppure (N, 7) in radianti.

    Returns:
        (flange_T, J) con shape (N, 4, 4) e (N, 6, 7).
    """
    q = np.atleast_2d(np.asarray(q, dtype=float))
    if q.shape[1] != 7:
        raise ValueError(f"q deve avere 7 colonne, ricevuto shape {q.shape}")
    n = q.shape[0]

    T = np.tile(np.eye(4), (n, 1, 1))
    z_axes = np.empty((n, 7, 3))
    origins = np.empty((n, 7, 3))
    for i, (a, d, alpha) in enumerate(_DH):
        T = T @ _dh_transform(q[:, i], a, d, alpha)
        z_axes[:, i, :] = T[:, :3, 2]
        origins[:, i, :] = T[:, :3, 3]

    flange = T @ _dh_transform(np.zeros(n), 0.0, FLANGE_OFFSET, 0.0)
    p_e = flange[:, :3, 3]

    jv = np.cross(z_axes, p_e[:, None, :] - origins)  # (N, 7, 3)
    J = np.concatenate([jv, z_axes], axis=2).transpose(0, 2, 1)  # (N, 6, 7)
    return flange, J


def jacobian_singular_values(q: np.ndarray) -> np.ndarray:
    """Valori singolari (N, 6) del Jacobiano geometrico, in ordine decrescente."""
    _, J = forward_kinematics_and_jacobian(q)
    return np.linalg.svd(J, compute_uv=False)
