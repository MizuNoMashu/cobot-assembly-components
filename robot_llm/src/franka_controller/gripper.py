"""
Controller per il gripper del robot Franka usando pylibfranka.Gripper.
"""

from typing import Optional, Dict, Any
import pylibfranka


class GripperController:
    """
    Controller per gestire il gripper del robot Franka con pylibfranka.

    Fornisce funzioni per:
    - Homing del gripper
    - Apertura/chiusura
    - Afferrare oggetti con controllo di forza
    - Leggere lo stato del gripper
    """

    def __init__(self, robot_ip: str):
        """
        Inizializza il controller del gripper.

        Args:
            robot_ip: Indirizzo IP del robot Franka
        """
        self.robot_ip = robot_ip
        self._gripper: Optional[pylibfranka.Gripper] = None
        self._is_homed = False
        self._connect()

    def _connect(self) -> None:
        """Stabilisce la connessione con il gripper."""
        try:
            self._gripper = pylibfranka.Gripper(self.robot_ip)
            print(f"✓ Connesso al gripper Franka")
        except Exception as e:
            raise ConnectionError(f"Impossibile connettersi al gripper: {e}")

    @property
    def gripper(self) -> pylibfranka.Gripper:
        """Restituisce l'oggetto Gripper di pylibfranka."""
        if self._gripper is None:
            raise RuntimeError("Gripper non connesso")
        return self._gripper

    def homing(self) -> bool:
        """
        Esegue l'homing del gripper per calibrazione.

        Returns:
            True se l'homing è completato con successo
        """
        try:
            print("Esecuzione homing del gripper...")
            self.gripper.homing()
            self._is_homed = True
            print("✓ Homing completato")
            return True
        except Exception as e:
            print(f"✗ Errore durante homing: {e}")
            return False

    def open(self, width: float = 0.08, speed: float = 0.1) -> bool:
        """
        Apre il gripper alla larghezza specificata.

        Args:
            width: Larghezza target in metri (default: 0.08m = 80mm, max ~0.08m)
            speed: Velocità di apertura in m/s (default: 0.1 m/s)

        Returns:
            True se l'apertura è completata con successo
        """
        if not self._is_homed:
            print("⚠ Gripper non ancora inizializzato. Esecuzione homing...")
            if not self.homing():
                return False

        try:
            print(f"Apertura gripper a {width * 1000:.1f}mm con velocità {speed} m/s")
            result = self.gripper.move(width, speed)
            print("✓ Gripper aperto")
            return result
        except Exception as e:
            print(f"✗ Errore durante apertura: {e}")
            return False

    def close(self, speed: float = 0.1) -> bool:
        """
        Chiude il gripper completamente.

        Args:
            speed: Velocità di chiusura in m/s

        Returns:
            True se la chiusura è completata con successo
        """
        return self.open(width=0.0, speed=speed)

    def grasp(
        self,
        width: float,
        force: float = 60.0,
        speed: float = 0.1,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
    ) -> bool:
        """
        Afferra un oggetto con larghezza e forza specificate.

        Args:
            width: Larghezza dell'oggetto in metri
            force: Forza di presa in Newton (default: 60N)
            speed: Velocità di movimento in m/s
            epsilon_inner: Tolleranza interna in metri (default: 0.005m)
            epsilon_outer: Tolleranza esterna in metri (default: 0.005m)

        Returns:
            True se l'oggetto è stato afferrato con successo
        """
        if not self._is_homed:
            print("⚠ Gripper non ancora inizializzato. Esecuzione homing...")
            if not self.homing():
                return False

        try:
            print(f"Tentativo di presa: larghezza={width * 1000:.1f}mm, forza={force}N")

            # pylibfranka.Gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer)
            success = self.gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer)

            if success:
                print("✓ Oggetto afferrato con successo")
            else:
                print("✗ Presa fallita - oggetto non rilevato")

            return success

        except Exception as e:
            print(f"✗ Errore durante la presa: {e}")
            return False

    def release(self, speed: float = 0.1) -> bool:
        """
        Rilascia l'oggetto afferrato aprendo il gripper.

        Args:
            speed: Velocità di apertura in m/s

        Returns:
            True se il rilascio è completato con successo
        """
        print("Rilascio oggetto...")
        return self.open(width=0.08, speed=speed)

    def get_state(self) -> Optional[Dict[str, Any]]:
        """
        Ottiene lo stato corrente del gripper usando read_once().

        Returns:
            Dict con informazioni su larghezza, temperatura, ecc., oppure None se errore
        """
        try:
            state = self.gripper.read_once()
            # pylibfranka.GripperState ha attributi: width, max_width, is_grasped, temperature, time
            return {
                "width": state.width if hasattr(state, "width") else 0.0,
                "max_width": state.max_width if hasattr(state, "max_width") else 0.08,
                "is_grasped": state.is_grasped if hasattr(state, "is_grasped") else False,
                "temperature": state.temperature if hasattr(state, "temperature") else 0.0,
            }
        except Exception as e:
            print(f"✗ Errore nella lettura dello stato: {e}")
            return None

    def print_state(self) -> None:
        """Stampa lo stato corrente del gripper in modo formattato."""
        state = self.get_state()
        if state is None:
            print("Impossibile leggere lo stato del gripper")
            return

        print("\n=== Stato Gripper Franka ===")
        print(f"Larghezza corrente: {state['width'] * 1000:.1f} mm")
        print(f"Larghezza massima: {state['max_width'] * 1000:.1f} mm")
        print(f"Temperatura: {state['temperature']:.1f} °C")
        print(f"Oggetto afferrato: {'Sì' if state['is_grasped'] else 'No'}")
        print("=" * 30)

    def is_grasping(self) -> bool:
        """
        Verifica se il gripper sta attualmente afferrando un oggetto.

        Returns:
            True se un oggetto è afferrato, False altrimenti
        """
        state = self.get_state()
        return state["is_grasped"] if state else False

    def get_server_version(self) -> str:
        """
        Ottiene la versione del server del gripper.

        Returns:
            Stringa con la versione del server
        """
        try:
            return str(self.gripper.server_version())
        except Exception as e:
            print(f"✗ Errore nell'ottenere la versione: {e}")
            return "Unknown"

    def stop(self) -> bool:
        """
        Ferma il movimento corrente del gripper.

        Returns:
            True se il gripper è stato fermato con successo
        """
        try:
            self.gripper.stop()
            print("✓ Gripper fermato")
            return True
        except Exception as e:
            print(f"✗ Errore durante l'arresto: {e}")
            return False

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup se necessario."""
        if exc_type is not None:
            print(f"[Errore durante l'esecuzione: {exc_val}]")
        # pylibfranka gestisce automaticamente la disconnessione
        pass
