"""
Test unitari per i moduli franka_controller
"""
import pytest
import numpy as np
from franka_controller.utils import (
    minimum_jerk_trajectory
)


class TestUtils:
    """Test per le funzioni di utility"""
    
  
    def test_minimum_jerk_trajectory_start(self):
        """Test traiettoria minimum jerk all'inizio"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([1.0, 1.0, 1.0])
        duration = 5.0
        current_time = 0.0
        
        pos, vel, acc = minimum_jerk_trajectory(start, goal, duration, current_time)
        
        # All'inizio, dovremmo essere alla posizione di start
        np.testing.assert_array_almost_equal(pos, start)
        np.testing.assert_array_almost_equal(vel, np.zeros(3))
    
    def test_minimum_jerk_trajectory_end(self):
        """Test traiettoria minimum jerk alla fine"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([1.0, 1.0, 1.0])
        duration = 5.0
        current_time = 5.0
        
        pos, vel, acc = minimum_jerk_trajectory(start, goal, duration, current_time)
        
        # Alla fine, dovremmo essere alla posizione goal
        np.testing.assert_array_almost_equal(pos, goal)
        np.testing.assert_array_almost_equal(vel, np.zeros(3))
    
    def test_minimum_jerk_trajectory_middle(self):
        """Test traiettoria minimum jerk a metà"""
        start = np.array([0.0])
        goal = np.array([1.0])
        duration = 2.0
        current_time = 1.0
        
        pos, vel, acc = minimum_jerk_trajectory(start, goal, duration, current_time)
        
        # A metà, dovremmo essere circa a metà strada
        assert 0.4 < pos[0] < 0.6
        assert vel[0] > 0  # Velocità positiva


class TestRobotController:
    """Test per il controller del robot (mock-based)"""
    
    def test_robot_initialization(self):
        """Test inizializzazione controller robot"""
        # Questo test richiederebbe un mock di pylibfranka
        # Per ora è un placeholder
        pass
    
    def test_motion_controller_initialization(self):
        """Test inizializzazione motion controller"""
        # Placeholder - richiede mock
        pass


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
