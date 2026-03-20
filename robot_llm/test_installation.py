#!/usr/bin/env python3
"""
Script di test per verificare l'installazione completa del progetto robot_llm.
"""

import sys
import numpy as np
import pylibfranka
from franka_controller import FrankaRobot, MotionController, GripperController
from franka_controller import __version__
from franka_controller.utils import (
    validate_joint_positions,
    compute_minimum_jerk_trajectory,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    pose_to_transformation_matrix,
)


def test_imports():
    """Test importazione moduli."""
    print("=" * 60)
    print("TEST 1: Importazione moduli")
    print("=" * 60)
    print("✓ Tutti i moduli importati con successo\n")

    print("Versioni:")
    print(f"  Python: {sys.version.split()[0]}")
    print(f"  NumPy: {np.__version__}")
    print(f"  pylibfranka: {pylibfranka.__version__}")
    print(f"  franka_controller: {__version__}\n")

    print("Moduli disponibili:")
    print("  - FrankaRobot")
    print("  - MotionController")
    print("  - GripperController")
    print()


def test_joint_validation():
    """Test validazione posizioni giunti."""
    print("=" * 60)
    print("TEST 2: Validazione posizioni giunti")
    print("=" * 60)

    # Posizione home valida
    valid_pos = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    result = validate_joint_positions(valid_pos)
    print(f"Posizione home: {result}")
    assert result == True, "La posizione home dovrebbe essere valida"

    # Posizione con troppi elementi
    try:
        invalid_pos = [0.0] * 8
        validate_joint_positions(invalid_pos)
        assert False, "Dovrebbe generare errore per numero errato di giunti"
    except ValueError as e:
        print(f"✓ Errore corretto per numero giunti: {e}\n")


def test_trajectory():
    """Test generazione traiettoria minimum jerk."""
    print("=" * 60)
    print("TEST 3: Traiettoria minimum jerk")
    print("=" * 60)

    tau_values = [0.0, 0.25, 0.5, 0.75, 1.0]
    s_values = [compute_minimum_jerk_trajectory(t) for t in tau_values]

    print(f"Valori tau: {tau_values}")
    print(f"Valori s:   {[round(s, 3) for s in s_values]}")

    # Verifica condizioni al contorno
    assert abs(s_values[0] - 0.0) < 1e-10, "s(0) deve essere 0"
    assert abs(s_values[-1] - 1.0) < 1e-10, "s(1) deve essere 1"

    # Verifica monotonia crescente
    for i in range(len(s_values) - 1):
        assert s_values[i] < s_values[i + 1], "La traiettoria deve essere crescente"

    print("✓ Traiettoria corretta\n")


def test_rotation_matrices():
    """Test conversione Euler <-> Rotation matrix."""
    print("=" * 60)
    print("TEST 4: Conversioni matrici di rotazione")
    print("=" * 60)

    # Test Euler -> Rotation matrix
    roll, pitch, yaw = 0.1, 0.2, 0.3
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    print(f"Euler angles (rad): roll={roll}, pitch={pitch}, yaw={yaw}")
    print(f"Determinante: {np.linalg.det(R):.6f} (deve essere ~1.0)")
    print(f"R @ R.T ≈ I: {np.allclose(R @ R.T, np.eye(3))}")

    # Verifica proprietà matrice di rotazione
    assert abs(np.linalg.det(R) - 1.0) < 1e-10, "Det(R) deve essere 1"
    assert np.allclose(R @ R.T, np.eye(3)), "R deve essere ortogonale"

    # Test Rotation matrix -> Euler (round-trip)
    roll2, pitch2, yaw2 = rotation_matrix_to_euler(R)
    print(f"Euler angles ricostruiti: roll={roll2:.6f}, pitch={pitch2:.6f}, yaw={yaw2:.6f}")

    assert abs(roll - roll2) < 1e-6, "Roll deve essere preservato"
    assert abs(pitch - pitch2) < 1e-6, "Pitch deve essere preservato"
    assert abs(yaw - yaw2) < 1e-6, "Yaw deve essere preservato"

    print("✓ Conversioni corrette\n")


def test_transformation_matrices():
    """Test composizione matrici di trasformazione."""
    print("=" * 60)
    print("TEST 5: Matrici di trasformazione omogenee")
    print("=" * 60)

    position = [0.5, 0.3, 0.2]
    rotation = euler_to_rotation_matrix(0.1, 0.2, 0.3)

    # Crea matrice 4x4
    T = pose_to_transformation_matrix(position, rotation)

    print(f"Posizione: {position}")
    print("Matrice di trasformazione 4x4:")
    print(T)

    # Verifica dimensioni
    assert T.shape == (4, 4), "Deve essere 4x4"

    # Verifica ultima riga
    assert np.allclose(T[3, :], [0, 0, 0, 1]), "Ultima riga deve essere [0 0 0 1]"

    # Verifica blocco rotazione
    assert np.allclose(T[:3, :3], rotation), "Blocco rotazione preservato"

    # Verifica vettore traslazione
    assert np.allclose(T[:3, 3], position), "Vettore posizione preservato"

    print("✓ Matrice di trasformazione corretta\n")


def test_pylibfranka_types():
    """Test tipi disponibili in pylibfranka."""
    print("=" * 60)
    print("TEST 6: Tipi pylibfranka disponibili")
    print("=" * 60)

    types_list = [x for x in dir(pylibfranka) if not x.startswith("_")]
    print("Tipi disponibili:")
    for t in types_list:
        print(f"  - {t}")

    # Verifica tipi essenziali
    essential_types = ["Robot", "Gripper", "RobotState", "GripperState"]
    for t in essential_types:
        assert hasattr(pylibfranka, t), f"Tipo {t} deve essere disponibile"

    print(f"\n✓ {len(types_list)} tipi disponibili\n")


def main():
    """Esegue tutti i test."""
    print("\n" + "=" * 60)
    print("TEST INSTALLAZIONE ROBOT_LLM")
    print("=" * 60 + "\n")

    try:
        test_imports()
        test_joint_validation()
        test_trajectory()
        test_rotation_matrices()
        test_transformation_matrices()
        test_pylibfranka_types()

        print("=" * 60)
        print("✓ TUTTI I TEST COMPLETATI CON SUCCESSO")
        print("=" * 60)
        print("\nL'installazione è completa e funzionante!")
        print("\nProssimi passi:")
        print("  1. Esegui './run.sh' per testare la connessione al robot")
        print("  2. Usa 'python3 examples/main.py' per l'interfaccia interattiva")
        print("  3. Prova 'python3 examples/quick_examples.py' per esempi specifici")

        return 0

    except Exception as e:
        print(f"\n✗ TEST FALLITO: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
