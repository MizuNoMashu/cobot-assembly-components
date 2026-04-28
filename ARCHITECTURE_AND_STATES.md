# Franka Robot Controller Architecture & State Flows

## Overview

This document describes the complete state machine, operational flows, and key functions of the Franka Robot Controller system. The controller is organized into three main components:

1. **FrankaRobot** - Main robot interface and state manager
2. **MotionController** - Joint and Cartesian motion execution
3. **GripperController** - End-effector gripper control

---

## 1. State Machines

### 1.1 Robot Mode States

The robot operates in four distinct states managed by `RobotMode` enum:

```
DISCONNECTED ──connection──> READY ──command──> RUNNING ──complete──> READY
     ▲                          ▲                   │
     │                          │                   │
     └────error recovery────────┴───error occurs───> ERROR_LOCKED
```

#### State Descriptions

| State | Description | Allowed Operations | Next State |
|-------|-------------|-------------------|-----------|
| **DISCONNECTED** | Robot not connected or initialization failed | None | READY (via reconnection) |
| **READY** | Robot connected and ready for commands | `get_state()`, `set_collision_behavior()`, motion/gripper commands | RUNNING |
| **RUNNING** | Active motion or operation in progress | Read-only operations via state queries | READY |
| **ERROR_LOCKED** | Critical error occurred, all commands blocked | `get_state()`, error recovery functions | READY (via recovery) |

#### State Transitions

- **READY → RUNNING**: Triggered by motion commands (`move_to_joint_positions()`, `move_relative()`) or gripper operations
- **RUNNING → READY**: Motion completes successfully or timeout occurs
- **ANY → ERROR_LOCKED**: Unrecoverable error (connection loss, validation failure, real-time constraint violation)
- **ERROR_LOCKED → READY**: Manual error recovery or reconnection

---

### 1.2 Gripper Mode States

The gripper follows an identical state machine pattern:

```
DISCONNECTED ──connection──> READY ──command──> RUNNING ──complete──> READY
     ▲                          ▲                   │
     │                          │                   │
     └────reconnect────────────┴──error occurs────> ERROR_LOCKED
```

#### Gripper-Specific Considerations

- **Homing Required**: Gripper must be homed before `grasp()` or `open()` operations
- **Independent Error State**: Gripper errors do NOT automatically lock the robot; they are isolated
- **Always Stoppable**: `stop()` command is allowed even in ERROR_LOCKED state

---

## 2. Error Classification

### 2.1 Robot Error Types (`RobotErrorType`)

Errors are automatically classified by exception inspection:

| Error Type | Trigger Conditions | Severity | Recoverable |
|------------|-------------------|----------|-------------|
| **REALTIME** | "realtime", "priority", "scheduler" in message | CRITICAL | No |
| **CONNECTION** | "connect", "network" in message or connection loss | CRITICAL | No |
| **COMMUNICATION** | "timeout", "packet", "communication" in message | HIGH | Yes |
| **VALIDATION** | Invalid parameters (dimensions, ranges, types) | MEDIUM | Yes |
| **COMMAND** | "motion", "control", "command" in message | HIGH | Yes |
| **RECOVERY** | "recovery" in message | LOW | Yes |
| **UNKNOWN** | Unclassified exception | MEDIUM | Yes |

### 2.2 Gripper Error Types (`GripperErrorType`)

Similar classification specific to gripper operations:

| Error Type | Conditions | Recoverable |
|------------|-----------|-------------|
| **CONNECTION** | Network/connection failure | No |
| **COMMUNICATION** | Timeout, packet loss | Yes |
| **VALIDATION** | Width, speed, force out of range | Yes |
| **COMMAND** | Operation failure (move, grasp, stop) | Yes |
| **HOMING** | Homing procedure failed | Yes |
| **UNKNOWN** | Unclassified | Yes |

### 2.3 Error Data Structure

Each error is captured as a `RobotError` or `GripperError` dataclass:

```python
@dataclass
class RobotError:
    operation: str              # Function name that failed
    error_type: RobotErrorType  # Classification
    message: str                # Exception message
    timestamp: datetime         # When error occurred
    recoverable: bool           # Can recover automatically?
    original_exception: Exception  # Original Python exception
```

---

## 3. Control Flow Diagrams

### 3.1 Robot Initialization Flow

```
FrankaRobot.__init__()
    │
    ├─> _connect()
    │    ├─> Create pylibfranka.Robot(ip, realtime_config)
    │    ├─ Success: mode = READY
    │    └─ Failure: mode = ERROR_LOCKED, raise ConnectionError
    │
    ├─> Initialize MotionController(self)
    ├─> Initialize GripperController(ip)
    └─> Return: FrankaRobot instance ready for commands
```

### 3.2 Joint Motion Flow

```
move_to_joint_positions(target_positions, speed_factor, tolerance)
    │
    ├─> Validation
    │    ├─ _ensure_can_command() [check READY/ERROR_LOCKED]
    │    ├─ Validate 7 joint values target
    │    ├─ Validate speed_factor ∈ (0.0, 1.0]
    │    └─ Validate tolerance errors > 0
    │
    ├─> Read Initial State
    │    ├─ q_start = current joint positions
    │    └─ duration = 5.0 / speed_factor
    │       (Formula explanation: duration is inversely proportional to speed_factor.
    │        Higher speed_factor → shorter movement time.
    │        Examples: speed_factor=0.2 → 25s, speed_factor=1.0 → 5s)
    │        The base constant 5.0 sets the time scale for reference movements.
    │
    ├─> Enter RUNNING State
    │    └─ mode = RUNNING
    │
    ├─> Activate Joint Position Control Loop
    │    └─ active_control = start_joint_position_control()
    │       (This opens a real-time communication channel with the robot.
    │        The robot's control interface is now active and expects commands
    │        every millisecond. This is the "active control" phase.)
    │
    ├─> **CONTEXT: What happens in the 1kHz control loop**
    │    The robot runs at 1000 Hz (1 millisecond per cycle).
    │    For each cycle, the robot's onboard controller:
    │    - Reads current joint positions and forces (via sensors)
    │    - Checks for safety violations (collisions/contacts)
    │    - Waits for our command via readOnce()
    │    - Executes our command (new joint target)
    │    - Repeats next millisecond
    │
    │    Our code must provide a new command EVERY millisecond or the 
    │    connection times out. That's why we can't do slow calculations
    │    inside the loop - we MUST respect the 1ms deadline.
    │
    ├─> Control Loop (repeated every 1ms)
    │    ├─ Read robot state
    │    │  (Get current joint positions, velocities, forces from sensors)
    │    │
    │    ├─ Check for cartesian_collision → STOP + ERROR
    │    │  (Collision = hard impact detected on end-effector
    │    │   Threshold: ~10N impact force in any direction
    │    │   Action: Set motion_finished=True immediately (HARD STOP)
    │    │   Reason: Hardware damage risk, must stop NOW)
    │    │
    │    ├─ Check for cartesian_contact → STOP + ERROR
    │    │  (Contact = gentle touch detected (lower threshold than collision)
    │    │   Threshold: ~5N force in any Cartesian axis
    │    │   Action: Set motion_finished=True immediately 
    │    │   Reason: Unexpected contact detected, safer to stop)
    │    │
    │    ├─ Calculate progress = time_elapsed / duration
    │    │  (Normalized progress from 0.0 to 1.0. Represents motion completion %)
    │    │  Example: At t=8.33s in a 25s motion → progress = 8.33/25 = 0.333 (33%)
    │    │
    │    ├─ Apply minimum jerk trajectory: s = f(progress)
    │    │  (Smooth interpolation coefficient using polynomial s(τ)=10τ³-15τ⁴+6τ⁵)
    │    │  s(0.333) = 10(0.333)³ - 15(0.333)⁴ + 6(0.333)⁵ ≈ 0.172
    │    │  This value smoothly goes from 0→1, NOT linearly!)
    │    │
    │    ├─ Interpolate: q_current = q_start + (target - q_start) × s
    │    │  (Linear combination: position smoothly transitions from start to target
    │    │   weighted by smooth trajectory coefficient s)
    │    │  Example: If q_start=[0], target=[2], s=0.172
    │    │           q_current = 0 + (2-0)×0.172 = 0.344 rad (17.2% of way there)
    │    │
    │    ├─ Send command: writeOnce(JointPositions(q_current))
    │    │  (Send EXACTLY this joint position to robot
    │    │   Robot will execute within its joint velocity limits
    │    │   MUST be done within 1ms or connection dies)
    │    │
    │    └─ If progress >= 1.0: motion_finished = True
    │       (When we've used up all the duration time, mark motion done
    │        Next iteration will exit the loop)
    │
    ├─> Read Final State
    │    └─ q_final = final joint positions
    │
    ├─> Validate Tolerance
    │    ├─ error = max(|target - q_final|)
    │    │  (Maximum absolute position difference across all 7 joints.
    │    │   Measures worst-case joint tracking error in radians.)
    │    ├─ If error ≤ tolerance: Success
    │    └─ If error > tolerance: Raise RuntimeError
    │
    ├─> Return to READY State
    │    └─ mode = READY
    │
    └─ Exception Handler
        └─ _enter_error_locked() → mode = ERROR_LOCKED, return False
```

### 3.3 Gripper Grasp Flow

```
gripper.grasp(width, force, speed, epsilon_inner, epsilon_outer)
    │
    ├─> Validation
    │    ├─ _ensure_can_command() [check READY/ERROR_LOCKED]
    │    ├─ Validate width ∈ [0.0, 0.08] m
    │    ├─ Validate speed > 0
    │    ├─ Validate force > 0
    │    └─ Validate epsilons ≥ 0
    │
    ├─> Check Homing Status
    │    ├─ If not homed: Execute homing()
    │    └─ If homing fails: Return False
    │
    ├─> Enter RUNNING State
    │    └─ mode = RUNNING
    │
    ├─> Execute Grasp
    │    └─ success = gripper.grasp(width, speed, force, ε_inner, ε_outer)
    │
    ├─> Return to READY State
    │    └─ mode = READY
    │
    ├─ On Success
    │    └─ Return True, object grasped
    │
    └─ On Failure
        ├─ If exception: _enter_error_locked()
        └─ Else: Return False (object not detected)
```

### 3.4 Error Recovery Flow

```
Scenario: Motion command fails → ERROR_LOCKED

Step 1: Error Detection
    └─ Exception raised in move_to_joint_positions()

Step 2: Error Classification
    ├─ _classify_error(exception)
    └─ Determine RobotErrorType and recoverable flag

Step 3: Enter ERROR_LOCKED
    ├─ _collect_error() → Store in self.errors[]
    ├─ _enter_error_locked() → Set mode = ERROR_LOCKED
    └─ Print error details

Step 4: Command Blocking
    ├─ All motion commands check _ensure_can_command()
    ├─ RaisesRuntimeError if ERROR_LOCKED
    └─ Users cannot issue new commands

Step 5: Recovery Options
    ├─ Option A: automatic_error_recovery()
    │            └─ Analyze stored errors
    │            └─ Attempt corrective actions
    │            └─ mode = READY if successful
    │
    └─ Option B: Manual reconnection
                └─ robot.reconnect()
                └─ Destroy robot instance
                └─ Reconnect to same IP
                └─ mode = READY if successful
```

---

## 4. Key Functions Reference

### 4.1 FrankaRobot Core Methods

#### Connection Management

| Function | Purpose | Returns | Raises |
|----------|---------|---------|--------|
| `_connect()` | Establish connection to Franka robot | None | ConnectionError |
| `get_state()` | Read current robot state (position, velocity, forces) | `RobotState` | RuntimeError |
| `robot` property | Access underlying pylibfranka.Robot instance | `pylibfranka.Robot` | RuntimeError |

#### Safety & Validation

| Function | Purpose | Returns | Raises |
|----------|---------|---------|--------|
| `_ensure_connected()` | Verify robot is connected | None | RuntimeError |
| `_ensure_can_command()` | Verify not in ERROR_LOCKED | None | RuntimeError |
| `_check_length()` | Validate array dimensions | None | ValueError |
| `set_collision_behavior(...)` | Configure collision thresholds | None | RuntimeError/ValueError |

#### State Queries

| Function | Purpose | Returns |
|----------|---------|---------|
| `get_current_joint_positions()` | Get q vector (7 joints) | `numpy.ndarray` |
| `get_current_cartesian_pose()` | Get 4×4 transformation matrix | `numpy.ndarray` |
| `get_current_cartesian_velocity()` | Get 6D velocity (linear + angular) | `numpy.ndarray` |
| `get_external_forces()` | Get 6D force/torque estimates | `numpy.ndarray` |
| `get_cartesian_collision()` | Check collision on 6D axes | `numpy.ndarray` (bool) |
| `get_cartesian_contact()` | Check contact on 6D axes | `numpy.ndarray` (bool) |

#### Error Handling

| Function | Purpose | Returns | Scope |
|----------|---------|---------|-------|
| `_classify_error(exception)` | Categorize exception into RobotErrorType | `RobotErrorType` | Private |
| `_collect_error(...)` | Record error in log and set last_error | `RobotError` | Private |
| `_enter_error_locked(...)` | Transition to ERROR_LOCKED, log error | `RobotError` | Private |
| `automatic_error_recovery()` | Attempt recovery from known error states | `bool` | Public |

---

### 4.2 MotionController Methods

#### Joint Motion

| Function | Purpose | Parameters | Returns |
|----------|---------|-----------|---------|
| `move_to_joint_positions()` | Move to absolute joint configuration | `target_positions`, `speed_factor`, `tolerance` | `bool` |
| `move_relative()` | Apply delta to current joint positions | `delta_positions`, `speed_factor` | `bool` |

#### Validation

| Function | Purpose |
|----------|---------|
| `_validate_joint_vector()` | Ensure exactly 7 joint values |
| `_validate_speed_factor()` | Check speed_factor ∈ (0.0, 1.0] |
| `_validate_duration()` | Ensure positive duration |

#### Trajectory Generation

- **Minimum Jerk Trajectory**: Applied during motion execution
  - Formula: `s(τ) = 10τ³ - 15τ⁴ + 6τ⁵` where τ = progress ∈ [0, 1]
  - **Why this formula?** 
    - 5th-order polynomial (quintic) ensures continuous acceleration
    - Derivative (velocity): `ṡ(τ) = 30τ² - 60τ³ + 30τ⁴` → smooth velocity profile
    - 2nd derivative (acceleration): `s̈(τ) = 60τ - 180τ² + 120τ³` → smooth acceleration
    - Minimizes jerk (3rd derivative) by design
  - Benefits: Smoother motion, less mechanical stress, better force control

#### Motion Monitoring

Per-iteration loop checks:
1. Cartesian collision detection
2. Cartesian contact detection
3. Progress calculation: `progress = elapsed_time / duration`
4. Trajectory interpolation
5. Command write-once per cycle

---

### 4.3 GripperController Methods

#### Initialization & Recovery

| Function | Purpose | Returns | Notes |
|----------|---------|---------|-------|
| `homing()` | Initialize gripper, calibrate limits | `bool` | Must be called before grasp/open |
| `reconnect()` | Reset and reconnect gripper hardware | None | Full hardware reinit |
| `reset_error_lock()` | Clear ERROR_LOCKED flag (software only) | None | Does NOT reset hardware |

#### Motion Commands

| Function | Purpose | Parameters | Returns |
|----------|---------|-----------|---------|
| `open()` | Move gripper to specified width | `width` (0.0-0.08m), `speed` | `bool` |
| `close()` | Close gripper to width=0.0 | `speed` | `bool` |
| `grasp()` | Grasp object with force control | `width`, `force`, `speed`, `ε_inner`, `ε_outer` | `bool` |
| `release()` | Release grasped object (full open) | `speed` | `bool` |
| `stop()` | Emergency stop (allowed in ERROR_LOCKED) | None | `bool` |

#### State Queries

| Function | Purpose | Returns |
|----------|---------|---------|
| `get_state()` | Read gripper status | `Dict` with width, max_width, is_grasped, temperature |

#### Validation

| Function | Purpose |
|----------|---------|
| `_validate_width()` | Check width ∈ [0.0, 0.08] |
| `_validate_speed()` | Check speed > 0 |
| `_validate_force()` | Check force > 0 |

---

### 4.4 Utility Functions

#### Trajectory Computation

| Function | Purpose | Formula | Returns |
|----------|---------|---------|---------|
| `compute_minimum_jerk_trajectory(progress)` | Calculate smooth interpolation coefficient | `s(τ) = 10τ³ - 15τ⁴ + 6τ⁵` | `float ∈ [0, 1]` |
| | | τ = np.clip(progress, 0.0, 1.0) | |
| | | **Boundary conditions:** s(0)=0, s(1)=1, ṡ(0)=ṡ(1)=0, s̈(0)=s̈(1)=0 | |
| `minimum_jerk_trajectory(start, goal, duration, time)` | Generate full trajectory with velocities | Minimum jerk polynomial | `(pos, vel, acc)` tuple |
| `validate_joint_positions(positions)` | Verify 7-joint configuration | Length check + bounds | `bool` |

---

## 5. Operational Workflows

### 5.1 Typical Pick-and-Place Sequence

```python
# 1. Initialize
robot = FrankaRobot(robot_ip="172.16.0.2", enforce_realtime=True)
assert robot.mode == RobotMode.READY

# 2. Move to pick location
robot.motion.move_to_joint_positions(
    target_positions=[q1, q2, q3, q4, q5, q6, q7],
    speed_factor=0.3,
    tolerance=0.04
)
assert robot.mode == RobotMode.READY

# 3. Grasp object
success = robot.gripper.grasp(
    width=0.05,
    force=60.0,
    speed=0.1
)
assert success

# 4. Move to place location
robot.motion.move_to_joint_positions(
    target_positions=[q1', q2', q3', q4', q5', q6', q7'],
    speed_factor=0.3
)

# 5. Release
robot.gripper.release(speed=0.1)

# 6. Return to home
robot.motion.move_to_joint_positions(
    target_positions=[0, 0, 0, -π/2, 0, π/2, π/4],
    speed_factor=0.2
)
```

### 5.2 Error Recovery Sequence

```python
try:
    robot.motion.move_to_joint_positions([...])
except RuntimeError as e:
    # Robot is now in ERROR_LOCKED
    assert robot.mode == RobotMode.ERROR_LOCKED
    
    # Option 1: Automatic recovery
    if robot.last_error.recoverable:
        robot.automatic_error_recovery()
    
    # Option 2: Manual reconnection
    else:
        robot.reconnect()
        assert robot.mode == RobotMode.READY
    
    # Retry
    robot.motion.move_to_joint_positions([...])
```

### 5.3 State Polling Pattern

```python
# Continuous monitoring
while True:
    if robot.mode == RobotMode.RUNNING:
        state = robot.get_state()
        print(f"Position: {state.q}, Forces: {state.K_F_ext_hat_K}")
    
    elif robot.mode == RobotMode.ERROR_LOCKED:
        print(f"ERROR: {robot.last_error.message}")
        # Handle error
    
    elif robot.mode == RobotMode.READY:
        # Execute next command
        pass
    
    time.sleep(0.01)  # 10ms polling
```

---

## 6. Safety Considerations

### 6.1 Real-Time Constraint

- Motion control loop runs **every 1 millisecond**
- Commands must complete within deadline or trigger REALTIME error
- `enforce_realtime=True` (default) enforces strict timing
- Set to `False` only for development/testing

### 6.2 Collision Detection

- **Cartesian collision**: Detects impact on end-effector
- **Cartesian contact**: Detects gentle contact (lower threshold)
- Both stop motion immediately via `motion_finished = True`
- Thresholds configured via `set_collision_behavior()`

### 6.3 Force/Torque Monitoring

- Available via `get_external_forces()` (6D vector)
- Useful for compliance-based tasks
- Configure thresholds for reactive behaviors

### 6.4 Gripper Safety

- Gripper **must be homed** before grasp operations
- Homing calibrates finger limits and detects objects
- `epsilon_inner` / `epsilon_outer` control grasp tolerance
- Force limit prevents crushing objects

---

## 7. System Architecture Diagram

```
┌─────────────────────────────────────────────────────┐
│          User Application / Script                   │
│         (examples/main.py, etc.)                     │
└──────────────────────┬──────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
   ┌─────────────┐ ┌──────────────┐ ┌────────────────┐
   │FrankaRobot  │ │ Motion       │ │ Gripper        │
   │             │ │ Controller   │ │ Controller     │
   │ ┌─────────┐ │ │              │ │                │
   │ │ state   │ │ │ ┌──────────┐ │ │ ┌────────────┐ │
   │ │ errors  │ │ │ │motion    │ │ │ │ error      │ │
   │ │ mode    │ │ │ │loop      │ │ │ │ handling   │ │
   │ │         │ │ │ │          │ │ │ │ state mgmt │ │
   │ └─────────┘ │ │ └──────────┘ │ │ └────────────┘ │
   └────┬────────┘ └──────┬───────┘ └────────┬───────┘
        │                 │                  │
        └─────────────────┼──────────────────┘
                          │
                    ┌─────▼──────┐
                    │ pylibfranka│
                    │ (libfranka)│
                    └─────┬──────┘
                          │
                    ┌─────▼──────────────┐
                    │  Franka Robot      │
                    │  (Hardware)        │
                    └────────────────────┘
```

---

## 8. Configuration Parameters

### Robot Configuration

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `robot_ip` | "172.16.0.2" | Valid IP | Network address of robot |
| `enforce_realtime` | `True` | bool | Strict real-time scheduling |

### Motion Configuration

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `speed_factor` | 0.2 | (0.0, 1.0] | Motion velocity scaling |
| `tolerance` | 0.04 rad | (0, ∞) | Final position tolerance |
| `duration` | 5.0 / speed_factor | (0, ∞) | Motion time budget |

### Gripper Configuration

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `width` | 0.08 m | [0.0, 0.08] | Gripper opening distance |
| `speed` | 0.1 m/s | (0, ∞) | Gripper motion velocity |
| `force` | 60.0 N | (0, ∞) | Grasp force limit |
| `epsilon_inner` | 0.005 m | [0, ∞) | Inner grasp tolerance |
| `epsilon_outer` | 0.005 m | [0, ∞) | Outer grasp tolerance |

### Collision Behavior (Default Values)

```python
# Lower and upper torque thresholds for 7 joints
lower_torque_thresholds = [10, 10, 10, 10, 10, 10, 10]  # N⋅m
upper_torque_thresholds = [10, 10, 10, 10, 10, 10, 10]  # N⋅m

# Lower and upper force thresholds for 6 Cartesian directions
lower_force_thresholds = [10, 10, 10, 10, 10, 10]        # N
upper_force_thresholds = [10, 10, 10, 10, 10, 10]        # N
```

---

## 9. Troubleshooting Guide

### Issue: REALTIME Error

**Symptom**: `RobotErrorType.REALTIME` / "priority", "scheduler" message

**Cause**: Motion loop exceeded 1ms deadline

**Solution**:
1. Reduce `speed_factor` to longer duration
2. Close other applications consuming CPU
3. Set `enforce_realtime=False` for testing only

### Issue: CONNECTION Error

**Symptom**: Cannot connect to robot IP

**Cause**: Network unreachable, robot powered off, wrong IP

**Solution**:
1. Verify robot IP matches actual hardware
2. Check network connectivity: `ping 172.16.0.2`
3. Ensure robot powered on and emergency stop cleared

### Issue: Motion Fails (ERROR_LOCKED)

**Symptom**: After motion command, mode becomes ERROR_LOCKED

**Cause**: Collision, invalid parameters, hardware issue

**Solution**:
1. Check `robot.last_error.message` for details
2. If recoverable: call `automatic_error_recovery()`
3. If not: call `reconnect()` and retry

### Issue: Grasp Fails

**Symptom**: `grasp()` returns `False`

**Cause**: Object too far apart, gripper not homed, insufficient force

**Solution**:
1. Verify gripper homing: `gripper.homing()`
2. Adjust width/force parameters
3. Check object position tolerance

---

## 10. Data Flow Example: Motion Execution

```
User Command: move_to_joint_positions([q1, ..., q7], 0.3, 0.04)
       │
       ├─ Validation
       │   ├─ _ensure_can_command() ✓
       │   ├─ Validate 7 joints ✓
       │   └─ Validate speed_factor ∈ (0, 1] ✓
       │
       ├─ Read Initial State
       │   ├─ q_start = [1.234, -0.567, ..., 0.123]
       │   └─ duration = 5.0 / 0.3 = 16.67 seconds
       │
       ├─ Activate Control Loop
       │   ├─ Call start_joint_position_control()
       │   └─ mode = RUNNING
       │
       ├─ Iteration 1 (t=0ms)
       │   ├─ Read state ✓
       │   ├─ Check collision ✓
       │   ├─ progress = 0 / 16670 = 0.0%
       │   ├─ s = 10(0)³ - 15(0)⁴ + 6(0)⁵ = 0.0
       │   ├─ q_cmd = q_start + (target - q_start) × 0.0 = q_start
       │   └─ Write command ✓
       │
       ├─ Iteration 2-500 (t=1-500ms)
       │   ├─ progress gradually increases
       │   ├─ s smoothly increases from 0 to 1
       │   └─ q_cmd interpolates smoothly toward target
       │
       ├─ Iteration 16668 (t=16668ms)
       │   ├─ progress = 16668 / 16670 = 99.99%
       │   ├─ s ≈ 1.0
       │   └─ q_cmd ≈ target
       │
       ├─ Iteration 16670 (t=16670ms)
       │   ├─ progress = 16670 / 16670 = 100.0%
       │   ├─ q_cmd = target
       │   ├─ motion_finished = True
       │   └─ Exit loop
       │
       ├─ Read Final State
       │   ├─ q_final = [1.233, -0.566, ..., 0.124]
       │   └─ error = max(|target - q_final|) = 0.0123 rad
       │
       ├─ Tolerance Check
       │   ├─ 0.0123 ≤ 0.04? YES ✓
       │   └─ Success
       │
       ├─ Cleanup
       │   └─ mode = READY
       │
       └─ Return: True
```

---

## 11. Formula Reference & Mathematical Explanations

### 11.1 Motion Duration Formula

**Formula:** $\text{duration} = \frac{5.0}{\text{speed\_factor}}$

**Explanation:**
- Creates **inverse proportionality** between speed and time
- Larger `speed_factor` → shorter duration (faster movement)
- Base constant `5.0` defines the reference time scale (in seconds)

**Examples:**
| speed_factor | duration | Movement Profile |
|--------------|----------|-----------------|
| 0.1 | 50.0s | Very slow, precise |
| 0.2 | 25.0s | Default, smooth |
| 0.5 | 10.0s | Medium-fast |
| 1.0 | 5.0s | Maximum speed |

**Use case:** Higher values for delicate operations, lower for fast pick-and-place.

---

### 11.2 Progress Normalization

**Formula:** $\text{progress} = \frac{\text{time\_elapsed}}{\text{duration}}$

**Explanation:**
- Normalizes elapsed time into [0, 1] range
- 0.0 = movement start, 1.0 = movement complete
- Drives the trajectory interpolation algorithm

**Example trace:**
```
time_elapsed=0ms    → progress=0.0%
time_elapsed=5s     → progress=20.0% (5/25)
time_elapsed=12.5s  → progress=50.0% (12.5/25)
time_elapsed=25s    → progress=100.0% (25/25) ✓ Complete
```

---

### 11.3 Minimum Jerk Trajectory Polynomial

**Formula:** $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$ where $\tau \in [0, 1]$

**Why this specific polynomial?**

1. **Boundary Conditions** (ensures smooth start/stop):
   - $s(0) = 0$ (start at initial position)
   - $s(1) = 1$ (reach target position)
   - $\dot{s}(0) = 0$ (start with zero velocity)
   - $\dot{s}(1) = 0$ (stop at target)
   - $\ddot{s}(0) = 0$ (start with zero acceleration)
   - $\ddot{s}(1) = 0$ (stop smoothly)

2. **Derivative (Velocity Profile):**
   $$\dot{s}(\tau) = 30\tau^2 - 60\tau^3 + 30\tau^4$$
   - Smooth bell curve, peaks at ~50% progress
   - Zero at start and end = no sudden velocity changes

3. **Second Derivative (Acceleration):**
   $$\ddot{s}(\tau) = 60\tau - 180\tau^2 + 120\tau^3$$
   - Smooth S-curve, symmetric around τ=0.5
   - Gradually ramps up then down = reduced mechanical stress

4. **Third Derivative (Jerk - rate of acceleration change):**
   $$\dddot{s}(\tau) = 60 - 360\tau + 360\tau^2$$
   - Minimized by design ← This is why it's called "minimum jerk"
   - Reduces sudden force transients on robot joints

**Practical benefits:**
- Smoother motion → better servo tracking
- Lower peak forces → less collision risk
- Reduced vibration → better precision
- More comfortable motion (for human collaboration)

**Comparison with alternatives:**
| Method | Smoothness | Peak Force | Comfort |
|--------|-----------|-----------|---------|
| Linear (bang-bang) | Poor | Very High | Jerky |
| Trapezoidal profile | Moderate | High | Abrupt |
| **Minimum Jerk** | **Excellent** | **Lower** | **Smooth** |
| Spline (higher order) | Excellent | Lower | Smoother but slower |

---

### 11.4 Position Interpolation

**Formula:** $\mathbf{q}_{\text{current}} = \mathbf{q}_{\text{start}} + (\mathbf{q}_{\text{target}} - \mathbf{q}_{\text{start}}) \times s(\tau)$

**Breakdown:**
- $\mathbf{q}_{\text{start}}$: Initial 7-joint vector (radians)
- $\mathbf{q}_{\text{target}}$: Goal 7-joint vector (radians)
- $s(\tau)$: Smooth coefficient from minimum jerk trajectory [0, 1]
- $(...)$: Delta vector = total distance to travel per joint

**Element-wise for joint $i$:**
$$q_i^{\text{cmd}} = q_i^{\text{start}} + (q_i^{\text{target}} - q_i^{\text{start}}) \cdot s(\tau)$$

**Example for a single joint:**
```
q_start = 0.5 rad
q_target = 2.5 rad
delta = 2.5 - 0.5 = 2.0 rad

At τ=0.0:   q_cmd = 0.5 + 2.0 × 0.0 = 0.5 rad (start)
At τ=0.5:   q_cmd = 0.5 + 2.0 × 0.79 = 1.98 rad (near middle, not exactly!)
At τ=1.0:   q_cmd = 0.5 + 2.0 × 1.0 = 2.5 rad (target)
```

*Note: Due to minimum jerk, τ=0.5 doesn't mean exactly 50% position - it means 50% time with accelerated motion*

---

### 11.5 Position Error Tolerance Check

**Formula:** $\text{error} = \max_i |q_i^{\text{target}} - q_i^{\text{final}}|$

**Meaning:**
- Finds the **worst-case joint** (maximum absolute deviation)
- Measured in radians for each of 7 joints
- Only one joint needs to exceed tolerance to fail

**Example:**
```
target       = [0.0, -π/2, 0.5, π, 0.2, 0.1, 0.0]
final_actual = [0.001, -1.565, 0.502, 3.145, 0.195, 0.105, 0.002]
difference   = [0.001, -0.008, 0.002, 0.004, -0.005, 0.005, 0.002]
abs_diff     = [0.001, 0.008, 0.002, 0.004, 0.005, 0.005, 0.002]
error        = max([...]) = 0.008 rad

tolerance = 0.04 rad
0.008 ≤ 0.04? YES ✓ Movement accepted
```

**Typical tolerance values:**
- High precision (assembly): 0.01-0.02 rad (~0.6-1.2°)
- General pick-and-place: 0.04-0.06 rad (~2.3-3.4°)
- Loose tolerance (quick motion): 0.1+ rad

---

### 11.6 Duration Time Constant Rationale

**Why base = 5.0 seconds?**

The choice of `5.0` as the base constant is empirical:

1. **Hardware constraints:**
   - Franka Panda max joint velocity: ~2 rad/s
   - Typical motion: ~1-2 rad
   - Natural timescale: 1-2 seconds minimum

2. **Real-time loop frequency:**
   - Control loop runs at 1 kHz (1000 Hz)
   - 5 seconds = 5000 control cycles
   - Sufficient granularity for smooth interpolation

3. **Trade-off:**
   - Too small: Robot jitters from quantization
   - Too large: Movements become sluggish
   - 5.0s is a well-balanced reference for collaborative robots

4. **Flexibility:**
   - Users scale via `speed_factor` parameter
   - `speed_factor=1.0` gives fastest safe motion (5s baseline)
   - `speed_factor=0.2` gives 8x longer motion (25s) for delicate tasks

---

## Summary Table: Function Categories

| Category | Key Functions | Purpose |
|----------|---------------|---------|
| **Initialization** | `__init__()`, `_connect()` | Setup robot instance |
| **Motion** | `move_to_joint_positions()`, `move_relative()` | Execute motion |
| **Gripper** | `grasp()`, `open()`, `close()`, `release()` | End-effector control |
| **State** | `get_state()`, `get_current_joint_positions()` | Read robot status |
| **Safety** | `_ensure_can_command()`, `set_collision_behavior()` | Prevent unsafe operations |
| **Error Handling** | `_classify_error()`, `_enter_error_locked()`, `automatic_error_recovery()` | Fault management |
| **Validation** | `_validate_*()` methods | Parameter checking |

---

**Document Version**: 1.0  
**Last Updated**: April 2026  
**Architecture**: Franka Panda + libfranka + pylibfranka bindings
