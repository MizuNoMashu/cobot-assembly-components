"""
Controllo di forza ibrido (forza/posizione) per inserimento e avvitamento.

pylibfranka non ha una modalità "force control" pronta: l'unico modo per
applicare una forza è il controllo in coppia (start_torque_control), dove si
comanda tau = J^T · wrench (+ Coriolis). La gravità è già compensata dal robot.

Questo modulo contiene la logica pura (solo numpy, nessun accesso al robot) di
un controllore ibrido lungo un asse di spinta `a` (di default l'asse z del
tool, cioè la direzione di avvicinamento):

  - lungo `a`:        forza controllata (feed-forward + PI sulla forza misurata)
  - ortogonale ad `a`: impedenza cartesiana (tiene la posizione, cede un po'
                       per autoallinearsi durante l'inserimento)
  - orientamento:     impedenza, con eventuale rotazione attorno ad `a`
                       (avvitamento) con coppia saturata a `max_torque`

Fasi: BIAS (azzera l'offset della stima di forza) → APPROACH (avvicinamento a
velocità limitata fino al contatto) → RAMP (la forza sale in modo smooth) →
RUN (mantenimento / inserimento / avvitamento con condizioni di arresto) →
RELEASE (la forza torna a zero in modo smooth prima di terminare).

Il robot non usa mai rampe istantanee: sia l'ingresso sia l'uscita dalla
forza sono smooth, e la derivata della coppia comandata è limitata.

Convenzione dei segni (libfranka): O_F_ext_hat_K è il wrench che l'ambiente
esercita sul robot. Spingendo contro una superficie lungo `a`, la superficie
risponde con una forza lungo -a: la forza di spinta misurata è quindi
-(f_ext · a).
"""

import math
from dataclasses import dataclass, field, fields
from enum import Enum
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from .kinematics import FR3_Q_MAX, FR3_Q_MIN
from .utils import compute_minimum_jerk_trajectory

MAX_FORCE_N = 40.0
MAX_TORQUE_NM = 6.0
MODES = ("hold", "insert", "screw")


class ForceTaskFailed(Exception):
    """L'obiettivo non è stato raggiunto (blocco, timeout, nessun contatto…).

    Il controllo è terminato in modo ordinato (forza riportata a zero): il
    robot non è bloccato e non entra in ERROR_LOCKED.
    """

    def __init__(self, reason: str, result: Optional[Dict[str, Any]] = None):
        super().__init__(reason)
        self.result = result or {}


class ForceTaskAbort(RuntimeError):
    """Violazione di sicurezza (forza eccessiva): arresto immediato."""


class Phase(str, Enum):
    BIAS = "bias"
    APPROACH = "approach"
    RAMP = "ramp"
    RUN = "run"
    RELEASE = "release"


# ---------------------------------------------------------------------------
# Utilità di geometria
# ---------------------------------------------------------------------------


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def rotation_vector(R: np.ndarray) -> np.ndarray:
    """Vettore di rotazione (asse·angolo) di una matrice di rotazione."""
    cos_t = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(cos_t)
    skew = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        return 0.5 * skew
    if math.pi - theta < 1e-3:
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(axis))
        signs = np.sign(R[i, :] + R[:, i])
        signs[i] = 1.0
        axis = axis * np.where(signs == 0, 1.0, signs)
        return theta * axis / np.linalg.norm(axis)
    return skew / (2.0 * math.sin(theta)) * theta


def as_jacobian(raw: Any) -> np.ndarray:
    """Normalizza l'output di Model.zero_jacobian a una matrice (6, 7).

    libfranka restituisce 42 valori in ordine column-major; se pylibfranka lo
    espone già come array (6, 7) viene lasciato invariato.
    """
    arr = np.asarray(raw, dtype=float)
    if arr.shape == (6, 7):
        return arr
    if arr.size == 42:
        return arr.reshape((6, 7), order="F")
    raise ValueError(f"Jacobiano con forma inattesa: {arr.shape}")


# ---------------------------------------------------------------------------
# Parametri
# ---------------------------------------------------------------------------


@dataclass
class ForceTaskParams:
    mode: str
    force: float                                  # N, spinta lungo l'asse
    axis_tool: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    # obiettivi
    hold_time: float = 5.0                        # s (hold)
    target_depth: Optional[float] = None          # m dal punto di contatto
    angle: float = 0.0                            # rad con segno (screw)
    angular_speed: float = 0.3                    # rad/s (picco)
    max_torque: float = 2.0                       # Nm attorno all'asse (screw)
    stop_at_torque: bool = False                  # coppia massima = obiettivo

    # avvicinamento e limiti
    approach_speed: float = 0.01                  # m/s, 0 = già a contatto
    max_approach_distance: float = 0.05           # m
    contact_threshold: float = 1.5                # N
    max_time: float = 30.0                        # s in fase RUN
    max_depth: float = 0.05                       # m dal contatto
    stall_time: float = 1.5                       # s senza avanzamento (insert)
    max_lateral_deviation: float = 0.02           # m
    max_lateral_force: float = 15.0               # N

    # temporizzazioni
    bias_time: float = 0.3
    ramp_time: float = 1.0
    release_time: float = 0.5

    # guadagni
    lateral_stiffness: float = 300.0              # N/m
    rotational_stiffness: float = 30.0            # Nm/rad
    kp_force: float = 0.2
    ki_force: float = 0.8                         # 1/s
    integral_limit: float = 8.0                   # N
    max_velocity_axial: float = 0.03              # m/s (limita la corsa libera)
    nullspace_stiffness: float = 10.0
    max_torque_step: float = 0.8                  # Nm per ciclo a 1 kHz

    # sicurezza
    abort_force: Optional[float] = None           # N, default max(2F, F+15) ≤ 80
    joint_margin: float = 0.05                    # rad dai limiti di giunto

    def __post_init__(self) -> None:
        self.axis_tool = tuple(float(v) for v in self.axis_tool)
        if self.abort_force is None:
            self.abort_force = min(max(2.0 * self.force, self.force + 15.0), 80.0)
        self.validate()

    @property
    def axis_unit(self) -> np.ndarray:
        axis = np.asarray(self.axis_tool, dtype=float)
        return axis / np.linalg.norm(axis)

    def validate(self) -> None:
        def finite(name: str, value: float) -> None:
            if not math.isfinite(value):
                raise ValueError(f"{name} deve essere un numero finito.")

        if self.mode not in MODES:
            raise ValueError(f"mode deve essere uno tra {MODES}.")
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float):
                finite(f.name, value)
        if len(self.axis_tool) != 3 or np.linalg.norm(self.axis_tool) < 1e-6:
            raise ValueError("axis deve essere un vettore 3D non nullo.")
        if not (0.0 < self.force <= MAX_FORCE_N):
            raise ValueError(f"force deve essere in (0, {MAX_FORCE_N}] N.")
        if self.mode == "hold" and not (0.0 < self.hold_time <= 600.0):
            raise ValueError("hold_time deve essere in (0, 600] s.")
        if self.mode == "insert" and self.target_depth is None:
            raise ValueError("target_depth è obbligatorio per l'inserimento.")
        if self.target_depth is not None and not (0.0 < self.target_depth <= 0.2):
            raise ValueError("target_depth deve essere in (0, 0.2] m.")
        if self.mode == "screw":
            if self.angle == 0.0 or abs(self.angle) > 2.0 * math.pi * 3.0:
                raise ValueError("angle deve essere diverso da 0 e |angle| <= 6π rad.")
            if not (0.0 < self.angular_speed <= 1.0):
                raise ValueError("angular_speed deve essere in (0, 1] rad/s.")
            if not (0.0 < self.max_torque <= MAX_TORQUE_NM):
                raise ValueError(f"max_torque deve essere in (0, {MAX_TORQUE_NM}] Nm.")
        if not (0.0 <= self.approach_speed <= 0.05):
            raise ValueError("approach_speed deve essere in [0, 0.05] m/s.")
        if not (0.0 < self.max_approach_distance <= 0.3):
            raise ValueError("max_approach_distance deve essere in (0, 0.3] m.")
        if not (0.0 < self.max_time <= 600.0):
            raise ValueError("max_time deve essere in (0, 600] s.")
        if not (0.0 < self.max_depth <= 0.3):
            raise ValueError("max_depth deve essere in (0, 0.3] m.")
        if self.target_depth is not None and self.target_depth > self.max_depth:
            raise ValueError("target_depth non può superare max_depth.")
        if not (0.0 < self.lateral_stiffness <= 2000.0):
            raise ValueError("lateral_stiffness deve essere in (0, 2000] N/m.")
        if not (0.0 < self.rotational_stiffness <= 200.0):
            raise ValueError("rotational_stiffness deve essere in (0, 200] Nm/rad.")
        if not (0.0 < self.max_velocity_axial <= 0.1):
            raise ValueError("max_velocity_axial deve essere in (0, 0.1] m/s.")
        for name in ("ramp_time", "release_time", "bias_time"):
            if not (0.05 <= getattr(self, name) <= 20.0):
                raise ValueError(f"{name} deve essere in [0.05, 20] s.")
        if not (0.0 < self.max_torque_step <= 1.0):
            raise ValueError("max_torque_step deve essere in (0, 1] Nm/ciclo.")
        if not (0.0 <= self.kp_force <= 1.0 and 0.0 <= self.ki_force <= 5.0):
            raise ValueError("kp_force deve essere in [0, 1] e ki_force in [0, 5].")
        if not (0.0 < self.contact_threshold < self.force):
            raise ValueError("contact_threshold deve essere positivo e minore di force.")
        if not (0.0 < self.max_lateral_deviation <= 0.1 and 0.0 < self.max_lateral_force <= 40.0):
            raise ValueError("max_lateral_deviation in (0, 0.1] m e max_lateral_force in (0, 40] N.")
        if not (0.1 <= self.stall_time <= 60.0):
            raise ValueError("stall_time deve essere in [0.1, 60] s.")
        if self.abort_force <= self.force:
            raise ValueError("abort_force deve essere maggiore di force.")

    # soglie di collisione di libfranka da impostare durante il task: quelle di
    # default del robot (10 N) farebbero scattare il reflex appena si spinge.
    def collision_thresholds(self) -> Dict[str, list]:
        force_thr = self.abort_force + 10.0
        torque_thr = max(10.0, 2.0 * self.max_torque + 5.0)
        joint = [40.0, 40.0, 40.0, 40.0, 25.0, 25.0, 25.0]
        return {
            "lower_torque_thresholds": joint,
            "upper_torque_thresholds": joint,
            "lower_force_thresholds": [force_thr] * 3 + [torque_thr] * 3,
            "upper_force_thresholds": [force_thr] * 3 + [torque_thr] * 3,
        }


_ACCEPTED = {f.name for f in fields(ForceTaskParams)}


def make_params(mode: str, force: float, **kwargs: Any) -> ForceTaskParams:
    """Costruisce e valida i parametri; ignora chiavi sconosciute con errore esplicito."""
    unknown = set(kwargs) - _ACCEPTED
    if unknown:
        raise ValueError(f"parametri non riconosciuti: {sorted(unknown)}")
    clean = {k: v for k, v in kwargs.items() if v is not None}
    return ForceTaskParams(mode=mode, force=float(force), **clean)


def precheck_wrist_range(params: ForceTaskParams, q: np.ndarray) -> None:
    """Se si ruota attorno all'asse z del tool, la rotazione è del solo giunto 7."""
    if params.mode != "screw":
        return
    axis = params.axis_unit
    if not np.allclose(axis, [0.0, 0.0, 1.0], atol=1e-6):
        return
    target = q[6] + params.angle
    lo, hi = FR3_Q_MIN[6] + params.joint_margin, FR3_Q_MAX[6] - params.joint_margin
    if not (lo <= target <= hi):
        available = (hi - q[6]) if params.angle > 0 else (q[6] - lo)
        raise ValueError(
            f"Rotazione di {params.angle:.2f} rad oltre il limite del giunto 7 "
            f"(q7={q[6]:.2f}, target {target:.2f}, consentito [{lo:.2f}, {hi:.2f}]): "
            f"in questo verso restano {available:.2f} rad. Ripresa della vite "
            "(regrasp) necessaria per più giri."
        )


# ---------------------------------------------------------------------------
# Avvitamento a più passate (ripresa della vite tra una passata e l'altra)
# ---------------------------------------------------------------------------

REGRASP_MODES = ("gripper", "retract", "none")
MAX_PASSES = 20


@dataclass
class RegraspConfig:
    """Cosa fare tra una passata di avvitamento e la successiva.

    Il giunto 7 ha un'escursione limitata: tra due passate va riportato
    indietro a vuoto, senza svitare la vite. Dipende dall'utensile:

      - "gripper": la vite è tenuta dal gripper → apri, riporta indietro il
                   giunto 7, richiudi sulla vite (senza muoversi in assiale);
      - "retract": l'utensile è un cacciavite/bit → arretra lungo l'asse di
                   `retract_distance`, riporta indietro il giunto 7; la passata
                   successiva si riavvicina fino al contatto;
      - "none":    solo riavvolgimento del giunto 7 (utensile a cricchetto).
    """

    mode: str = "gripper"
    rewind_speed: float = 1.0                     # rad/s, picco, riavvolgimento a vuoto
    retract_distance: float = 0.005               # m (solo "retract")
    open_width_mm: float = 80.0                   # apertura del gripper (solo "gripper")
    grasp_width_mm: Optional[float] = None        # larghezza della presa (solo "gripper")
    grasp_force: float = 40.0                     # N (solo "gripper")
    gripper_speed: float = 0.05                   # m/s (solo "gripper")

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.mode not in REGRASP_MODES:
            raise ValueError(f"regrasp_mode deve essere uno tra {REGRASP_MODES}.")
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{f.name} deve essere un numero finito.")
        if not (0.05 <= self.rewind_speed <= 2.0):
            raise ValueError("rewind_speed deve essere in [0.05, 2] rad/s.")
        if self.mode == "retract" and not (0.001 <= self.retract_distance <= 0.05):
            raise ValueError("retract_distance deve essere in [0.001, 0.05] m.")
        if self.mode == "gripper":
            if self.grasp_width_mm is None:
                raise ValueError("grasp_width_mm è obbligatorio con regrasp_mode 'gripper'.")
            if not (0.0 <= self.grasp_width_mm <= 80.0 and 0.0 <= self.open_width_mm <= 80.0):
                raise ValueError("grasp_width_mm e open_width_mm devono essere in [0, 80] mm.")
            if self.grasp_width_mm >= self.open_width_mm:
                raise ValueError("open_width_mm deve essere maggiore di grasp_width_mm.")
            if not (0.0 < self.grasp_force <= 140.0):
                raise ValueError("grasp_force deve essere in (0, 140] N.")
            if not (0.0 < self.gripper_speed <= 0.1):
                raise ValueError("gripper_speed deve essere in (0, 0.1] m/s.")


_REGRASP_FIELDS = {f.name for f in fields(RegraspConfig)}


def make_regrasp(**kwargs: Any) -> RegraspConfig:
    """Costruisce e valida la configurazione; accetta `regrasp_mode` come alias di `mode`."""
    if "regrasp_mode" in kwargs:
        kwargs["mode"] = kwargs.pop("regrasp_mode")
    unknown = set(kwargs) - _REGRASP_FIELDS
    if unknown:
        raise ValueError(f"parametri di ripresa non riconosciuti: {sorted(unknown)}")
    return RegraspConfig(**{k: v for k, v in kwargs.items() if v is not None})


def wrist_rewind_target(params: ForceTaskParams) -> float:
    """Posizione del giunto 7 da cui ripartire: l'estremo opposto al verso di avvitamento."""
    if params.angle > 0:
        return float(FR3_Q_MIN[6] + params.joint_margin + 0.02)
    return float(FR3_Q_MAX[6] - params.joint_margin - 0.02)


def precheck_screw_passes(
    params: ForceTaskParams, regrasp: RegraspConfig, iterations: int, q: np.ndarray
) -> None:
    """Valida in anticipo un avvitamento a più passate (prima di muovere il robot)."""
    if not (isinstance(iterations, int) and 1 <= iterations <= MAX_PASSES):
        raise ValueError(f"iterations deve essere un intero in [1, {MAX_PASSES}].")
    precheck_wrist_range(params, q)
    if iterations == 1:
        return
    if not np.allclose(params.axis_unit, [0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError("Più passate sono supportate solo con asse [0, 0, 1] (rotazione del giunto 7).")
    lo = FR3_Q_MIN[6] + params.joint_margin
    hi = FR3_Q_MAX[6] - params.joint_margin
    if abs(params.angle) > (hi - lo) - 0.04:
        raise ValueError(
            f"Una passata di {abs(params.angle):.2f} rad non ci sta nell'escursione del giunto 7 "
            f"({hi - lo - 0.04:.2f} rad disponibili): riduci l'angolo per passata."
        )
    if regrasp.mode == "retract":
        if params.approach_speed <= 0.0:
            raise ValueError("Con regrasp_mode 'retract' serve approach_speed > 0 per riavvicinarsi.")
        if params.max_approach_distance <= regrasp.retract_distance:
            raise ValueError("max_approach_distance deve superare retract_distance.")


# ---------------------------------------------------------------------------
# Controllore
# ---------------------------------------------------------------------------


@dataclass
class StepOutput:
    tau: np.ndarray
    finished: bool
    info: Dict[str, Any] = field(default_factory=dict)


class ForceTaskController:
    """Calcola, ciclo per ciclo, la coppia da inviare in torque control."""

    def __init__(self, params: ForceTaskParams, q0: np.ndarray, x0: np.ndarray, R0: np.ndarray):
        self.p = params
        self.q0 = np.asarray(q0, dtype=float)
        self.x_hold = np.asarray(x0, dtype=float).copy()
        self.R0 = np.asarray(R0, dtype=float).copy()
        self.a = R0 @ params.axis_unit

        self.phase = Phase.BIAS
        self.x_start = self.x_hold.copy()
        self.x_contact = self.x_hold.copy()
        self.bias_f = 0.0
        self.bias_m = 0.0
        self._bias_sum_f = 0.0
        self._bias_sum_m = 0.0
        self._bias_n = 0

        self.integral = 0.0
        self.velocity_integral = 0.0
        self.theta_ref = 0.0
        self.theta_actual = 0.0

        self._phase_t0 = 0.0
        self._established_since: Optional[float] = None
        self._rotation_t0: Optional[float] = None
        self._best_depth = 0.0
        self._best_depth_t = 0.0
        self._torque_sat_since: Optional[float] = None
        self._force_at_release = 0.0
        self._release_ref_theta = 0.0

        self._snapshot: Dict[str, Any] = {}
        self.success: Optional[bool] = None
        self.reason: str = ""
        self.torque_reached = False
        self.last_force = 0.0
        self.last_depth = 0.0
        self.last_torque = 0.0

    # -- fasi -------------------------------------------------------------

    def _enter(self, phase: Phase, t: float) -> None:
        self.phase = phase
        self._phase_t0 = t

    def _begin_release(self, t: float, success: bool, reason: str) -> None:
        if self.phase is Phase.RELEASE:
            return
        self.success = success
        self.reason = reason
        self._force_at_release = max(self.last_force, 0.0)
        self._snapshot = {
            "depth_m": round(self.last_depth, 5),
            "angle_rad": round(self.theta_actual, 4),
            "final_force_n": round(self.last_force, 3),
            "final_torque_nm": round(self.last_torque, 3),
        }
        self.theta_ref = self.theta_actual  # niente coppia residua dopo il rilascio
        self._enter(Phase.RELEASE, t)

    def result(self) -> Dict[str, Any]:
        return {
            "success": bool(self.success),
            "reason": self.reason,
            "torque_reached": self.torque_reached,
            **self._snapshot,
        }

    # -- passo di controllo ----------------------------------------------

    def step(
        self,
        t: float,
        dt: float,
        q: np.ndarray,
        dq: np.ndarray,
        x: np.ndarray,
        R: np.ndarray,
        J: np.ndarray,
        coriolis: np.ndarray,
        ext_wrench: np.ndarray,
        tau_prev: np.ndarray,
    ) -> StepOutput:
        p, a = self.p, self.a
        twist = J @ dq
        v, w = twist[:3], twist[3:]
        v_a = float(v @ a)
        f_along = float(ext_wrench[:3] @ a)
        m_along = float(ext_wrench[3:] @ a)

        # --- offset della stima di forza (robot fermo, nessuna forza comandata)
        if self.phase is Phase.BIAS:
            self._bias_sum_f += f_along
            self._bias_sum_m += m_along
            self._bias_n += 1
            if t >= p.bias_time and self._bias_n > 0:
                self.bias_f = self._bias_sum_f / self._bias_n
                self.bias_m = self._bias_sum_m / self._bias_n
                self.x_start = x.copy()
                self.x_contact = x.copy()
                self._enter(Phase.APPROACH if p.approach_speed > 0.0 else Phase.RAMP, t)

        force_meas = -(f_along - self.bias_f)
        torque_meas = -(m_along - self.bias_m)
        depth = float((x - self.x_contact) @ a)
        lateral = x - self.x_hold
        lateral = lateral - (lateral @ a) * a
        self.last_force, self.last_depth, self.last_torque = force_meas, depth, torque_meas

        # --- controlli di sicurezza
        if force_meas > p.abort_force:
            raise ForceTaskAbort(
                f"Forza di contatto {force_meas:.1f} N oltre il limite di sicurezza "
                f"{p.abort_force:.1f} N."
            )
        if self.phase is not Phase.RELEASE:
            margin = p.joint_margin
            if np.any(q < FR3_Q_MIN + margin) or np.any(q > FR3_Q_MAX - margin):
                self._begin_release(t, False, "un giunto è vicino al proprio limite")
            elif float(np.linalg.norm(lateral)) > p.max_lateral_deviation:
                self._begin_release(
                    t, False, f"deviazione laterale oltre {p.max_lateral_deviation * 1000:.0f} mm"
                )

        # --- forza desiderata e legge assiale per fase
        force_des = 0.0
        axial_cmd = 0.0
        use_force_law = False

        if self.phase is Phase.BIAS:
            pass
        elif self.phase is Phase.APPROACH:
            travelled = float((x - self.x_start) @ a)
            # PI sulla velocità: il solo termine proporzionale non vincerebbe l'attrito statico.
            v_err = p.approach_speed - v_a
            self.velocity_integral = float(
                np.clip(self.velocity_integral + 400.0 * v_err * dt, 0.0, p.force)
            )
            axial_cmd = float(np.clip(150.0 * v_err + self.velocity_integral, -p.force, p.force))
            if force_meas > p.contact_threshold:
                self.x_contact = x.copy()
                self.integral = 0.0
                self._enter(Phase.RAMP, t)
            elif travelled > p.max_approach_distance:
                self._begin_release(
                    t, False, f"nessun contatto entro {p.max_approach_distance * 1000:.0f} mm"
                )
        if self.phase is Phase.RAMP:
            s = compute_minimum_jerk_trajectory((t - self._phase_t0) / p.ramp_time)
            force_des = p.force * s
            use_force_law = True
            if t - self._phase_t0 >= p.ramp_time:
                self._enter(Phase.RUN, t)
                self._best_depth, self._best_depth_t = depth, t
        elif self.phase is Phase.RUN:
            force_des = p.force
            use_force_law = True
            self._run_checks(t, depth, force_meas)
        elif self.phase is Phase.RELEASE:
            s = compute_minimum_jerk_trajectory((t - self._phase_t0) / p.release_time)
            force_des = self._force_at_release * (1.0 - s)
            use_force_law = True

        if use_force_law:
            err = force_des - force_meas
            if self.phase is not Phase.RELEASE:
                self.integral = float(
                    np.clip(self.integral + p.ki_force * err * dt, -p.integral_limit, p.integral_limit)
                )
            f_cmd = force_des + p.kp_force * err + self.integral
            f_cmd = float(np.clip(f_cmd, 0.0, p.force * 1.5 + p.integral_limit))
            damping = float(np.clip(max(p.force, 1.0) / p.max_velocity_axial, 50.0, 800.0))
            axial_cmd = f_cmd - damping * v_a

        # --- impedenza laterale
        k_l = p.lateral_stiffness
        f_lat = -k_l * lateral - 2.0 * math.sqrt(k_l) * (v - v_a * a)
        norm = float(np.linalg.norm(f_lat))
        if norm > p.max_lateral_force:
            f_lat *= p.max_lateral_force / norm

        # --- orientamento (con eventuale rotazione attorno all'asse)
        self._update_rotation(t)
        R_d = rotation_about_axis(a, self.theta_ref) @ self.R0
        e_o = rotation_vector(R_d @ R.T)
        self.theta_actual = self.theta_ref - float(e_o @ a)
        k_o = p.rotational_stiffness
        moment = k_o * e_o - 2.0 * math.sqrt(k_o) * w
        m_ax_raw = float(moment @ a)
        m_ax = float(np.clip(m_ax_raw, -p.max_torque, p.max_torque))
        moment = moment - m_ax_raw * a + m_ax * a
        torque_saturated = p.mode == "screw" and abs(m_ax_raw) >= 0.98 * p.max_torque
        self._screw_checks(t, e_o, torque_saturated)

        # --- coppie
        wrench = np.concatenate([f_lat + a * axial_cmd, moment])
        tau_task = J.T @ wrench
        n_proj = np.eye(7) - J.T @ np.linalg.pinv(J.T)
        kn = p.nullspace_stiffness
        tau_null = n_proj @ (kn * (self.q0 - q) - 2.0 * math.sqrt(kn) * dq)
        tau = tau_task + tau_null + coriolis
        tau = tau_prev + np.clip(tau - tau_prev, -p.max_torque_step, p.max_torque_step)

        finished = self.phase is Phase.RELEASE and (t - self._phase_t0) >= p.release_time
        info = {
            "phase": self.phase.value,
            "force_measured": round(force_meas, 3),
            "force_desired": round(force_des, 3),
            "depth_m": round(depth, 5),
            "angle_rad": round(self.theta_actual, 4),
            "torque_axis_nm": round(torque_meas, 3),
            "lateral_mm": round(float(np.linalg.norm(lateral)) * 1000.0, 2),
        }
        return StepOutput(tau=tau, finished=finished, info=info)

    # -- condizioni di arresto -------------------------------------------

    def _run_checks(self, t: float, depth: float, force_meas: float) -> None:
        p = self.p
        run_t = t - self._phase_t0
        near_setpoint = abs(force_meas - p.force) <= max(0.15 * p.force, 0.5)
        if near_setpoint:
            if self._established_since is None:
                self._established_since = t
        else:
            self._established_since = None
        established = self._established_since is not None and (t - self._established_since) >= 0.3

        if depth > p.max_depth:
            self._begin_release(t, False, f"profondità massima {p.max_depth * 1000:.0f} mm superata")
            return
        if p.mode == "hold":
            if run_t >= p.hold_time:
                self._begin_release(t, True, "forza mantenuta per il tempo richiesto")
            return
        if run_t > p.max_time:
            self._begin_release(t, False, f"timeout di {p.max_time:.0f} s")
            return
        if p.mode == "insert":
            if depth >= p.target_depth - 1e-4:
                self._begin_release(t, True, f"profondità {depth * 1000:.1f} mm raggiunta")
            elif depth > self._best_depth + 3e-4:
                self._best_depth, self._best_depth_t = depth, t
            elif established and t - self._best_depth_t > p.stall_time:
                self._begin_release(
                    t, False,
                    f"inserimento bloccato a {depth * 1000:.1f} mm (obiettivo "
                    f"{p.target_depth * 1000:.1f} mm)",
                )
        elif p.mode == "screw":
            if self._rotation_t0 is None and established:
                self._rotation_t0 = t

    def _update_rotation(self, t: float) -> None:
        p = self.p
        if p.mode != "screw" or self.phase is not Phase.RUN or self._rotation_t0 is None:
            return
        duration = 1.875 * abs(p.angle) / p.angular_speed
        self.theta_ref = p.angle * compute_minimum_jerk_trajectory((t - self._rotation_t0) / duration)

    def _screw_checks(self, t: float, e_o: np.ndarray, torque_saturated: bool) -> None:
        p = self.p
        if p.mode != "screw" or self.phase is not Phase.RUN or self._rotation_t0 is None:
            return
        if torque_saturated:
            if self._torque_sat_since is None:
                self._torque_sat_since = t
            elif t - self._torque_sat_since > 0.5:
                if p.stop_at_torque:
                    self.torque_reached = True
                    self._begin_release(t, True, f"coppia di serraggio {p.max_torque:.2f} Nm raggiunta")
                else:
                    self._begin_release(
                        t, False,
                        f"coppia massima {p.max_torque:.2f} Nm raggiunta a "
                        f"{self.theta_actual:.2f} rad su {p.angle:.2f} rad",
                    )
                return
        else:
            self._torque_sat_since = None
        rotation_done = abs(p.angle - self.theta_ref) < 1e-6
        if rotation_done and abs(float(e_o @ self.a)) < 0.05:
            self._begin_release(t, True, f"rotazione di {p.angle:.2f} rad completata")
