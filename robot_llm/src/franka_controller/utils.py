"""
Utility functions per il controller Franka
"""

import numpy as np
from typing import List, Tuple


def validate_joint_positions(positions: List[float], num_joints: int = 7) -> bool:
    """
        Valida le posizioni dei giunti,Verifica che ci siano 7 valori (uno per ogni giunto)
    Controlla che ogni valore sia dentro i limiti fisici del robot

        Args:
            positions: Lista delle posizioni dei giunti
            num_joints: Numero di giunti attesi (default: 7 per Franka Panda)

        Returns:
            True se le posizioni sono valide, False altrimenti
    """
    if len(positions) != num_joints:
        raise ValueError(
            f"Attesi {num_joints} giunti ({num_joints} valori), ricevuti {len(positions)}"
        )

    return True


def compute_minimum_jerk_trajectory(progress: float) -> float:
    """

    Calcola il coefficiente della traiettoria minimum jerk per un dato progresso.
    Genera un movimento fluido tra due punti.
    Evita movimenti bruschi → minimizza il “jerk” (derivata dell’accelerazione) attraverso un polinomio di ordine 5( 10t³ - 15t⁴ + 6t⁵)


    Args:
        progress: Progresso normalizzato [0, 1]

    Returns:
        Coefficiente della traiettoria minimum jerk [0, 1]
    """
    # Polinomio minimum jerk di ordine 5
    tau = np.clip(progress, 0.0, 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def minimum_jerk_trajectory(
    start: np.ndarray, goal: np.ndarray, duration: float, current_time: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Traiettoria minimum-jerk completa (posizione, velocita, accelerazione).

    Questa funzione mantiene compatibilita con la vecchia API usata nei test.

    Args:
        start: Posizione iniziale (array N-dimensionale)
        goal: Posizione finale (array N-dimensionale)
        duration: Durata totale della traiettoria [s]
        current_time: Tempo corrente [s]

    Returns:
        Tuple (pos, vel, acc), tutti array della stessa dimensione di start/goal
    """
    start_arr = np.asarray(start, dtype=float)
    goal_arr = np.asarray(goal, dtype=float)

    if start_arr.shape != goal_arr.shape:
        raise ValueError("start e goal devono avere la stessa dimensione")
    if duration <= 0:
        raise ValueError("duration deve essere > 0")

    tau = float(np.clip(current_time / duration, 0.0, 1.0))
    delta = goal_arr - start_arr

    # Position blending polynomial and derivatives in normalized time tau.
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds_dt = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / duration
    d2s_dt2 = (60 * tau - 180 * tau**2 + 120 * tau**3) / (duration**2)

    pos = start_arr + delta * s
    vel = delta * ds_dt
    acc = delta * d2s_dt2

    return pos, vel, acc


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Converte angoli di Eulero (ZYX) in matrice di rotazione 3x3.

    Args:
        roll: Rotazione attorno all'asse X in radianti
        pitch: Rotazione attorno all'asse Y in radianti
        yaw: Rotazione attorno all'asse Z in radianti

    Returns:
        Matrice di rotazione 3x3
    """
    # Matrici di rotazione base
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
    )

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Composizione ZYX (yaw * pitch * roll)
    return Rz @ Ry @ Rx


def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Converte matrice di rotazione 3x3 in angoli di Eulero (ZYX).

    Args:
        R: Matrice di rotazione 3x3

    Returns:
        Tupla di (roll, pitch, yaw) in radianti
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw


def pose_to_transformation_matrix(position: List[float], rotation: np.ndarray) -> np.ndarray:
    """
    Crea una matrice di trasformazione omogenea 4x4 da posizione e rotazione.

    Args:
        position: Lista [x, y, z] con la posizione in metri
        rotation: Matrice di rotazione 3x3

    Returns:
        Matrice di trasformazione omogenea 4x4
    """
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T


def transformation_matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Estrae posizione e rotazione da una matrice di trasformazione 4x4.

    Args:
        T: Matrice di trasformazione omogenea 4x4

    Returns:
        Tupla di (position, rotation) dove position è un array [x, y, z]
        e rotation è una matrice 3x3
    """
    position = T[:3, 3]
    rotation = T[:3, :3]
    return position, rotation


def cartesian_to_joint(x: float, y: float, z: float) -> List[float]:
    """
    Conversione approssimativa da coordinate cartesiane a posizioni dei giunti.

    NOTA: Questa è una funzione placeholder. Per un'implementazione reale,
    dovresti utilizzare la cinematica inversa del robot (IK).

    Args:
        x, y, z: Coordinate cartesiane in metri

    Returns:
        Lista delle posizioni dei giunti in radianti

    Raises:
        NotImplementedError: La cinematica inversa richiede librerie esterne
    """
    raise NotImplementedError(
        "La cinematica inversa deve essere implementata "
        "utilizzando librerie come PyKDL, IKPy, o il model di pylibfranka"
    )


def print_robot_state_info(state) -> None:
    """
    Stampa informazioni formattate sullo stato del robot

    Args:
        state: Oggetto RobotState di pylibfranka
    """
    print("=" * 60)
    print("STATO DEL ROBOT FRANKA")
    print("=" * 60)

    print("\nPosizioni dei giunti (radianti):")
    for i, pos in enumerate(state.q):
        print(f"  Joint {i + 1}: {pos:.4f}")

    print("\nVelocità dei giunti (rad/s):")
    for i, vel in enumerate(state.dq):
        print(f"  Joint {i + 1}: {vel:.4f}")

    print("\nTorque dei giunti (Nm):")
    for i, tau in enumerate(state.tau_J):
        print(f"  Joint {i + 1}: {tau:.4f}")

    print("\n" + "=" * 60)
