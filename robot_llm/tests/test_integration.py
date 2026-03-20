"""
Test di integrazione per i moduli franka_controller

NOTA: Questi test verificano solo la logica dei moduli, NON richiedono
hardware reale o pylibfranka installato.
"""

import numpy as np
import pytest
from unittest.mock import Mock, patch

pytest.importorskip("pylibfranka")


def test_imports():
    """Verifica che i moduli possano essere importati"""
    from franka_controller import FrankaRobot, MotionController, GripperController

    assert FrankaRobot is not None
    assert MotionController is not None
    assert GripperController is not None


@patch("franka_controller.gripper.pylibfranka.Gripper")
@patch("franka_controller.robot.pylibfranka.Robot")
def test_robot_initialization(mock_robot, mock_gripper):
    """Test inizializzazione robot"""
    from franka_controller import FrankaRobot

    # Mock del robot pylibfranka
    mock_robot_instance = Mock()
    mock_robot.return_value = mock_robot_instance

    mock_gripper_instance = Mock()
    mock_gripper.return_value = mock_gripper_instance

    # Crea robot
    robot = FrankaRobot("172.16.0.2")

    # Verifica che Robot/Gripper siano stati chiamati con l'IP corretto
    mock_robot.assert_called_once_with("172.16.0.2")
    mock_gripper.assert_called_once_with("172.16.0.2")

    # Verifica che i controller siano stati inizializzati
    assert robot.motion is not None
    assert robot.gripper is not None


@patch("franka_controller.gripper.pylibfranka.Gripper")
@patch("franka_controller.robot.pylibfranka.Robot")
def test_get_joint_positions(mock_robot, mock_gripper):
    """Test lettura posizioni giunti"""
    from franka_controller import FrankaRobot

    # Mock dello stato
    mock_state = Mock()
    mock_state.q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    mock_robot_instance = Mock()
    mock_robot_instance.read_once.return_value = mock_state
    mock_robot.return_value = mock_robot_instance

    mock_gripper.return_value = Mock()

    robot = FrankaRobot("172.16.0.2")
    positions = robot.get_current_joint_positions()

    assert len(positions) == 7
    assert isinstance(positions, np.ndarray)
    np.testing.assert_array_equal(positions, mock_state.q)


@patch("franka_controller.gripper.pylibfranka.Gripper")
@patch("franka_controller.robot.pylibfranka.Robot")
def test_move_to_joint_positions(mock_robot, mock_gripper):
    """Test movimento verso posizioni target"""
    from franka_controller import FrankaRobot

    # Mock del robot
    mock_state = Mock()
    mock_state.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    mock_robot_instance = Mock()
    mock_robot_instance.read_once.return_value = mock_state
    mock_robot_instance.start_joint_position_control = Mock()
    mock_robot.return_value = mock_robot_instance

    mock_gripper.return_value = Mock()

    robot = FrankaRobot("172.16.0.2")

    # Test movimento
    target = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    success = robot.motion.move_to_joint_positions(target, speed_factor=0.2)

    # Verifica che il controllo posizione sia stato avviato
    mock_robot_instance.start_joint_position_control.assert_called_once()

    assert success is True


@patch("franka_controller.gripper.pylibfranka.Gripper")
@patch("franka_controller.robot.pylibfranka.Robot")
def test_gripper_homing(mock_robot, mock_gripper_class):
    """Test homing del gripper"""
    from franka_controller import FrankaRobot

    mock_robot.return_value = Mock()

    # Mock del gripper
    mock_gripper_instance = Mock()
    mock_gripper_instance.homing = Mock()
    mock_gripper_class.return_value = mock_gripper_instance

    robot = FrankaRobot("172.16.0.2")
    success = robot.gripper.homing()

    # Verifica che homing sia stato chiamato
    mock_gripper_instance.homing.assert_called_once()
    assert success is True


@patch("franka_controller.gripper.pylibfranka.Gripper")
@patch("franka_controller.robot.pylibfranka.Robot")
def test_gripper_grasp(mock_robot, mock_gripper_class):
    """Test presa con gripper"""
    from franka_controller import FrankaRobot

    mock_robot.return_value = Mock()

    # Mock del gripper con grasp che restituisce True
    mock_gripper_instance = Mock()
    mock_gripper_instance.homing = Mock()
    mock_gripper_instance.grasp = Mock(return_value=True)
    mock_gripper_class.return_value = mock_gripper_instance

    robot = FrankaRobot("172.16.0.2")
    robot.gripper._is_homed = True  # Simula homing completato

    success = robot.gripper.grasp(width=0.03, force=20.0)

    # Verifica parametri
    mock_gripper_instance.grasp.assert_called_once()
    call_args = mock_gripper_instance.grasp.call_args[0]
    assert call_args[0] == 0.03
    assert call_args[2] == 20.0
    assert success is True


def test_joint_position_validation():
    """Test validazione posizioni giunti"""
    from franka_controller.utils import validate_joint_positions

    # Posizioni valide
    valid_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    validate_joint_positions(valid_positions)  # Non dovrebbe sollevare eccezioni

    # Numero errato di giunti
    with pytest.raises(ValueError, match="7 valori"):
        validate_joint_positions([0.0, 0.0, 0.0])

    # Valore fuori range
    invalid_positions = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # > 2.8973
    with pytest.raises(ValueError, match="fuori dai limiti"):
        validate_joint_positions(invalid_positions)


def test_minimum_jerk_trajectory():
    """Test coefficiente traiettoria minimum jerk"""
    from franka_controller.utils import compute_minimum_jerk_trajectory

    # Estremi
    assert compute_minimum_jerk_trajectory(0.0) == 0.0
    assert compute_minimum_jerk_trajectory(1.0) == 1.0

    # Punto medio: vicino a 0.5
    mid = compute_minimum_jerk_trajectory(0.5)
    assert 0.49 < mid < 0.51

    # Clipping fuori dominio
    assert compute_minimum_jerk_trajectory(-1.0) == 0.0
    assert compute_minimum_jerk_trajectory(2.0) == 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
