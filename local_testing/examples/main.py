#!/usr/bin/env python3
"""
Esempio completo di utilizzo del controller Franka Robot.

Questo script fornisce un menu interattivo per testare tutte le funzionalità
del robot: connessione, movimento dei giunti, controllo del gripper, impedance control,
lettura dello stato, e workflow completi.
"""

import sys
import traceback
import numpy as np
from franka_controller import FrankaRobot,RobotError, RobotErrorType, RobotMode, GripperError, GripperErrorType, GripperMode

pose1 = [0.1601747338677177, -0.13314153275982454, -0.23210223978843075, -2.282551255319605, 1.4401673292125021, 1.7223496288987419, 0.227743905258227]
pose2 = [0.24331267448521512, -0.1008804753139512, -0.16947217878813267, -2.3149512770949685, 1.4401392707139342, 1.7219941665734733, 0.22701246033680494]
pose3 = [0.24604548034651855, 0.10079375949413885, -0.16943161981786944, -2.527100991549152, 1.4400407739126129, 1.720661255079275, 0.27097378511370995]
poses_default = {
    "1": pose1,
    "2": pose2,
    "3": pose3
}
def print_menu(robot=None):
    """Stampa il menu principale."""

    print("\n" + "=" * 60)
    print("MENU PRINCIPALE - FRANKA ROBOT CONTROLLER")
    print("=" * 60)

    if robot is not None:
        print(f"Robot mode:   {robot.mode.value}")

        if hasattr(robot, "gripper") and robot.gripper is not None:
            print(f"Gripper mode: {robot.gripper.mode.value}")

        if robot.last_error is not None:
            print(
                f"Last robot error: "
                f"{robot.last_error.error_type.value} | "
                f"{robot.last_error.operation}"
            )

        if hasattr(robot, "gripper") and robot.gripper.last_error is not None:
            print(
                f"Last gripper error: "
                f"{robot.gripper.last_error.error_type.value} | "
                f"{robot.gripper.last_error.operation}"
            )

    else:
        print("Robot mode:   not connected")
        print("Gripper mode: not connected")

    print("=" * 60)
    print("1. Connetti al robot")
    print("2. Mostra stato del robot")
    print("3. Mostra stato del gripper")
    print("4. Vai alla posizione home")
    print("5. Movimento dei giunti custom")
    print("6. Movimento relativo dei giunti")
    print("7. Controllo impedenza")
    print("8. Homing del gripper")
    print("9. Apri gripper")
    print("10. Chiudi gripper")
    print("11. Afferra oggetto")
    print("12. Rilascia oggetto")
    print("13. Workflow Pick-and-Place")
    print("14. Configura collision behavior")
    print("15. Recovery automatico robot")
    print("16. Mostra errori robot")
    print("17. Mostra errori gripper")
    print("18. Svuota errori robot")
    print("19. Svuota errori gripper")
    print("20. Reset software lock gripper")
    print("21. Riconnetti gripper")
    print("0. Esci")
    print("=" * 60)
def connect_robot(
    robot_ip: str = "172.16.0.3",
    enforce_realtime: bool = True,
) -> FrankaRobot | None:
    try:
        print(f"\n[Connessione al robot {robot_ip}...]")
        print(f"[Realtime enforce: {enforce_realtime}]")

        robot = FrankaRobot(
            robot_ip=robot_ip,
            enforce_realtime=enforce_realtime,
        )

        print("✓ Robot connesso con successo!")
        return robot

    except Exception as e:
        print(f"✗ Errore di connessione: {e}")
        traceback.print_exc()
        return None
def show_robot_errors(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    robot.print_errors()


def show_gripper_errors(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    robot.gripper.print_errors()


def clear_robot_errors(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    robot.clear_errors()


def clear_gripper_errors(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    robot.gripper.clear_errors()


def reset_gripper_lock(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        robot.gripper.reset_error_lock()
    except Exception as e:
        print(f"✗ Errore reset gripper lock: {e}")


def reconnect_gripper(robot: FrankaRobot):
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        robot.gripper.reconnect()
    except Exception as e:
        print(f"✗ Errore riconnessione gripper: {e}")

def show_robot_state(robot: FrankaRobot):
    """Mostra lo stato del robot."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        robot.print_state()
    except Exception as e:
        print(f"✗ Errore nella lettura dello stato: {e}")


def show_gripper_state(robot: FrankaRobot):
    """Mostra lo stato del gripper."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        robot.gripper.print_state()
    except Exception as e:
        print(f"✗ Errore nella lettura dello stato del gripper: {e}")


def go_to_home(robot: FrankaRobot):
    """Porta il robot alla posizione home."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        print("\n[Ritorno alla posizione home...]")
        robot.motion.go_to_home(speed_factor=0.2)
    except Exception as e:
        print(f"✗ Errore: {e}")


def move_joints_custom(robot: FrankaRobot):
    """Movimento custom dei giunti."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    print("\n[Movimento giunti custom]")
    print("Inserisci le posizioni target per i 7 giunti (in radianti):")
    print("Formato: q1 q2 q3 q4 q5 q6 q7")
    print("Esempio: 0 -0.785 0 -2.356 0 1.571 0.785")

    try:
        if input("Usare pose predefinite? (y/n): ").lower() == "y":
            print("\nPose predefinite:")
            print("1. Pose 1")
            print("2. Pose 2")
            print("3. Pose 3")
            pose_choice = input("Scegli una pose (1-3): ")
            if pose_choice in poses_default:
                positions = poses_default[pose_choice]
            else:
                print("✗ Scelta non valida, inserisci manualmente le posizioni.")
                user_input = input("Posizioni: ")
                positions = [float(x) for x in user_input.split()]

        else:
            user_input = input("Posizioni: ")
            positions = [float(x) for x in user_input.split()]

        if len(positions) != 7:
            print("✗ Devi specificare esattamente 7 posizioni!")
            return

        speed = float(input("Velocità (0.0-1.0, default 0.2): ") or "0.2")

        print(f"\n[Movimento verso: {np.round(positions, 3).tolist()}]")
        robot.motion.move_to_joint_positions(positions, speed_factor=speed)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def move_joints_relative(robot: FrankaRobot):
    """Movimento relativo dei giunti."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    print("\n[Movimento relativo giunti]")
    print("Inserisci i delta per i 7 giunti (in radianti):")
    print("Esempio: 0.1 0 0 0 0 0 0  (muove solo joint 1 di 0.1 rad)")

    try:
        user_input = input("Delta: ")
        deltas = [float(x) for x in user_input.split()]

        if len(deltas) != 7:
            print("✗ Devi specificare esattamente 7 delta!")
            return

        speed = float(input("Velocità (0.0-1.0, default 0.2): ") or "0.2")

        print(f"\n[Movimento relativo: {np.round(deltas, 3).tolist()}]")
        robot.motion.move_relative(deltas, speed_factor=speed)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def impedance_control(robot: FrankaRobot):
    """Controllo impedenza."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    print("\n[Controllo impedenza]")
    print("Inserisci le posizioni target per i 7 giunti:")

    try:
        user_input = input("Posizioni: ")
        positions = [float(x) for x in user_input.split()]

        if len(positions) != 7:
            print("✗ Devi specificare esattamente 7 posizioni!")
            return

        print("Stiffness giunti (opzionale, lascia vuoto per default):")
        stiff_input = input("Stiffness [7 valori]: ")

        stiffness = None
        if stiff_input.strip():
            stiffness = [float(x) for x in stiff_input.split()]
            if len(stiffness) != 7:
                print("✗ Devi specificare 7 valori di stiffness!")
                return

        duration = float(input("Durata (secondi, default 5.0): ") or "5.0")

        print(f"\n[Controllo impedenza verso: {np.round(positions, 3).tolist()}]")
        robot.motion.impedance_control(positions, stiffness=stiffness, duration=duration)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def gripper_homing(robot: FrankaRobot):
    """Homing del gripper."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        print("\n[Homing gripper...]")
        robot.gripper.homing()
    except Exception as e:
        print(f"✗ Errore: {e}")


def gripper_open(robot: FrankaRobot):
    """Apri gripper."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        width_mm = float(input("Larghezza apertura in mm (default 80): ") or float("80"))
        width_m = width_mm / 1000.0
        speed = float(input("Velocità in m/s (default 0.1): ") or "0.1")

        print(f"\n[Apertura gripper a {width_mm}mm...]")
        robot.gripper.open(width=width_m, speed=speed)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def gripper_close(robot: FrankaRobot):
    """Chiudi gripper."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        speed = float(input("Velocità in m/s (default 0.1): ") or "0.1")

        print("\n[Chiusura gripper...]")
        robot.gripper.close(speed=speed)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def gripper_grasp(robot: FrankaRobot):
    """Afferra un oggetto."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        width_mm = float(input("Larghezza oggetto in mm (es. 50): "))
        force = float(input("Forza presa in N (default 60): ") or "60")
        speed = float(input("Velocità in m/s (default 0.1): ") or "0.1")

        width_m = width_mm / 1000.0

        print(f"\n[Presa: larghezza={width_mm}mm, forza={force}N...]")
        robot.gripper.grasp(width=width_m, force=force, speed=speed)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def gripper_release(robot: FrankaRobot):
    """Rilascia oggetto."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        print("\n[Rilascio oggetto...]")
        robot.gripper.release()
    except Exception as e:
        print(f"✗ Errore: {e}")


def pick_and_place_workflow(robot: FrankaRobot):
    """Workflow completo di pick-and-place."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    print("\n[Workflow Pick-and-Place]")
    print("Questo workflow simula una sequenza di pick-and-place:")
    print("1. Va alla posizione home")
    print("2. Apre il gripper")
    print("3. Si sposta verso la posizione di pick (simulata)")
    print("4. Afferra l'oggetto")
    print("5. Si sposta verso la posizione di place (simulata)")
    print("6. Rilascia l'oggetto")
    print("7. Ritorna a home")

    if input("\nContinuare? (y/n): ").lower() != "y":
        return

    try:
        # Step 1: Home
        print("\n[Step 1/7: Home position...]")
        robot.motion.go_to_home(speed_factor=0.2)

        # Step 2: Open gripper
        print("\n[Step 2/7: Apertura gripper...]")
        robot.gripper.open(width=0.08, speed=0.1)

        # Step 3: Move to pick position (simulata)
        print("\n[Step 3/7: Movimento verso pick position...]")
        pick_position = [0.3, -0.5, 0, -2.0, 0, 1.5, 0.785]
        robot.motion.move_to_joint_positions(pick_position, speed_factor=0.15)

        # Step 4: Grasp
        print("\n[Step 4/7: Presa oggetto (50mm, 60N)...]")
        robot.gripper.grasp(width=0.05, force=60.0, speed=0.1)

        # Step 5: Move to place position (simulata)
        print("\n[Step 5/7: Movimento verso place position...]")
        place_position = [-0.3, -0.5, 0, -2.0, 0, 1.5, 0.785]
        robot.motion.move_to_joint_positions(place_position, speed_factor=0.15)

        # Step 6: Release
        print("\n[Step 6/7: Rilascio oggetto...]")
        robot.gripper.release()

        # Step 7: Home
        print("\n[Step 7/7: Ritorno a home...]")
        robot.motion.go_to_home(speed_factor=0.2)

        print("\n✓ Workflow completato con successo!")

    except Exception as e:
        print(f"✗ Errore durante il workflow: {e}")
        print("Tentativo di recovery automatico...")
        robot.automatic_error_recovery()


def configure_collision(robot: FrankaRobot):
    """Configura collision behavior."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    print("\n[Configura Collision Behavior]")
    print("Lascia vuoto per usare valori di default sicuri.")

    try:
        print("\nSoglie di torque inferiori (7 valori in Nm):")
        lower_torque_input = input("Lower torque thresholds: ")

        print("Soglie di torque superiori (7 valori in Nm):")
        upper_torque_input = input("Upper torque thresholds: ")

        print("Soglie di forza inferiori (6 valori in N, Nm):")
        lower_force_input = input("Lower force thresholds: ")

        print("Soglie di forza superiori (6 valori in N, Nm):")
        upper_force_input = input("Upper force thresholds: ")

        lower_torque = (
            [float(x) for x in lower_torque_input.split()] if lower_torque_input.strip() else None
        )
        upper_torque = (
            [float(x) for x in upper_torque_input.split()] if upper_torque_input.strip() else None
        )
        lower_force = (
            [float(x) for x in lower_force_input.split()] if lower_force_input.strip() else None
        )
        upper_force = (
            [float(x) for x in upper_force_input.split()] if upper_force_input.strip() else None
        )

        robot.set_collision_behavior(lower_torque, upper_torque, lower_force, upper_force)

    except ValueError as e:
        print(f"✗ Input non valido: {e}")
    except Exception as e:
        print(f"✗ Errore: {e}")


def automatic_recovery(robot: FrankaRobot):
    """Recovery automatico da errori."""
    if robot is None:
        print("✗ Robot non connesso!")
        return

    try:
        print("\n[Recovery automatico...]")
        robot.automatic_error_recovery()
    except Exception as e:
        print(f"✗ Errore: {e}")

def main():
    print("=" * 60)
    print("FRANKA ROBOT CONTROLLER - Menu Interattivo")
    print("=" * 60)

    default_ip = "172.16.0.3"
    robot_ip = input(f"Indirizzo IP del robot (default {default_ip}): ") or default_ip

    enforce_input = input("Usare kEnforce realtime? (Y/n): ").lower()
    enforce_realtime = enforce_input != "n"

    robot = None

    while True:
        print_menu(robot)

        try:
            choice = input("\nScegli un'opzione: ")

            if choice == "0":
                print("\n[Uscita...]")
                if robot is not None:
                    print("Disconnessione dal robot...")
                print("Arrivederci!")
                sys.exit(0)

            elif choice == "1":
                robot = connect_robot(
                    robot_ip=robot_ip,
                    enforce_realtime=enforce_realtime,
                )

            elif choice == "2":
                show_robot_state(robot)

            elif choice == "3":
                show_gripper_state(robot)

            elif choice == "4":
                go_to_home(robot)

            elif choice == "5":
                move_joints_custom(robot)

            elif choice == "6":
                move_joints_relative(robot)

            elif choice == "7":
                impedance_control(robot)

            elif choice == "8":
                gripper_homing(robot)

            elif choice == "9":
                gripper_open(robot)

            elif choice == "10":
                gripper_close(robot)

            elif choice == "11":
                gripper_grasp(robot)

            elif choice == "12":
                gripper_release(robot)

            elif choice == "13":
                pick_and_place_workflow(robot)

            elif choice == "14":
                configure_collision(robot)

            elif choice == "15":
                automatic_recovery(robot)

            elif choice == "16":
                show_robot_errors(robot)

            elif choice == "17":
                show_gripper_errors(robot)

            elif choice == "18":
                clear_robot_errors(robot)

            elif choice == "19":
                clear_gripper_errors(robot)

            elif choice == "20":
                reset_gripper_lock(robot)

            elif choice == "21":
                reconnect_gripper(robot)

            else:
                print("✗ Opzione non valida!")

        except KeyboardInterrupt:
            print("\n\n[Interruzione con Ctrl+C]")

            if robot is not None:
                try:
                    print("Tentativo stop gripper...")
                    robot.gripper.stop()
                except Exception:
                    pass

                print("Disconnessione dal robot...")

            print("Arrivederci!")
            sys.exit(0)

        except Exception as e:
            print(f"\n✗ Errore imprevisto: {e}")
            traceback.print_exc()
if __name__ == "__main__":
    main()
