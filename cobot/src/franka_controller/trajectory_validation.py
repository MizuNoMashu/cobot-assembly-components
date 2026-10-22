"""
Controllo pre-esecuzione di traiettorie articolari.

Viene eseguito PRIMA di inviare qualsiasi comando al robot e rifiuta le
traiettorie che:
  - escono dai limiti di giunto (con un margine di sicurezza);
  - richiedono velocità/accelerazioni di giunto oltre i limiti del FR3
    (scalati da un fattore di sicurezza);
  - passano troppo vicino a una singolarità cinematica (sigma_min del
    Jacobiano sotto soglia), controllata lungo il percorso campionato e non
    solo sui waypoint, perché l'interpolazione articolare tra due waypoint può
    attraversare configurazioni peggiori degli estremi.

Il controllo ha un budget di tempo (timeout_sec): se viene superato la
traiettoria viene rifiutata (fail-safe) invece di partire senza verifica.
I limiti e le soglie sono sovrascrivibili con variabili d'ambiente
(vedi ValidationLimits.from_env).
"""

import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .kinematics import (
    FR3_DQ_MAX,
    FR3_Q_MAX,
    FR3_Q_MIN,
    jacobian_singular_values,
)

# Picchi della legge minimum-jerk normalizzata (s(τ) = 10τ³ - 15τ⁴ + 6τ⁵).
MIN_JERK_PEAK_VELOCITY = 1.875
MIN_JERK_PEAK_ACCELERATION = 5.7735

# Limite di accelerazione dei giunti FR3 (rad/s²). Valore da datasheet Franka:
# verificarlo sulla propria versione di robot/firmware.
FR3_DDQ_MAX = np.full(7, 10.0)

_MAX_REPORTED_VIOLATIONS = 8
_SVD_CHUNK = 256


class TrajectoryValidationError(ValueError):
    """Traiettoria rifiutata dal controllo pre-esecuzione."""

    def __init__(self, message: str, report: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.report = report or {}


@dataclass
class ValidationLimits:
    velocity_fraction: float = 0.8
    acceleration_fraction: float = 0.8
    joint_margin: float = 0.02
    min_sigma: float = 0.02
    samples_per_segment: int = 10
    timeout_sec: float = 10.0

    @classmethod
    def from_env(cls) -> "ValidationLimits":
        def _f(name: str, default: float) -> float:
            raw = os.environ.get(name)
            if raw is None or raw == "":
                return default
            try:
                return float(raw)
            except ValueError:
                return default

        d = cls()
        return cls(
            velocity_fraction=_f("COBOT_PRECHECK_VELOCITY_FRACTION", d.velocity_fraction),
            acceleration_fraction=_f("COBOT_PRECHECK_ACCEL_FRACTION", d.acceleration_fraction),
            joint_margin=_f("COBOT_PRECHECK_JOINT_MARGIN", d.joint_margin),
            min_sigma=_f("COBOT_PRECHECK_MIN_SIGMA", d.min_sigma),
            samples_per_segment=int(_f("COBOT_PRECHECK_SAMPLES_PER_SEGMENT", d.samples_per_segment)),
            timeout_sec=_f("COBOT_PRECHECK_TIMEOUT_SEC", d.timeout_sec),
        )


class _Check:
    """Accumula violazioni e rispetta il budget di tempo del pre-check."""

    def __init__(self, limits: ValidationLimits):
        self.limits = limits
        self.started = time.monotonic()
        self.violations: List[str] = []
        self.violation_count = 0
        self.min_sigma: Optional[float] = None
        self.min_sigma_where: Optional[str] = None
        self.max_velocity_ratio = 0.0
        self.max_acceleration_ratio = 0.0
        self.num_samples = 0

    def fail(self, message: str) -> None:
        self.violation_count += 1
        if len(self.violations) < _MAX_REPORTED_VIOLATIONS:
            self.violations.append(message)

    def check_budget(self) -> None:
        if time.monotonic() - self.started > self.limits.timeout_sec:
            raise TrajectoryValidationError(
                f"Controllo pre-esecuzione oltre il tempo massimo "
                f"({self.limits.timeout_sec:.1f} s): traiettoria non verificata, "
                "movimento annullato.",
                self.report(),
            )

    def joint_limits(self, Q: np.ndarray, labels: Sequence[str]) -> None:
        lo = FR3_Q_MIN + self.limits.joint_margin
        hi = FR3_Q_MAX - self.limits.joint_margin
        bad = np.argwhere((Q < lo) | (Q > hi))
        for row, joint in bad:
            self.fail(
                f"{labels[row]}: giunto {joint + 1} = {Q[row, joint]:.3f} rad fuori da "
                f"[{lo[joint]:.3f}, {hi[joint]:.3f}] (limiti FR3 con margine "
                f"{self.limits.joint_margin:.3f} rad)"
            )
            if self.violation_count >= _MAX_REPORTED_VIOLATIONS:
                break

    def singularity(self, Q: np.ndarray, labels: Sequence[str]) -> None:
        for start in range(0, Q.shape[0], _SVD_CHUNK):
            self.check_budget()
            chunk = Q[start:start + _SVD_CHUNK]
            sigma_min = jacobian_singular_values(chunk)[:, -1]
            idx = int(np.argmin(sigma_min))
            value = float(sigma_min[idx])
            if self.min_sigma is None or value < self.min_sigma:
                self.min_sigma = value
                self.min_sigma_where = labels[start + idx]
            for i in np.flatnonzero(sigma_min < self.limits.min_sigma):
                self.fail(
                    f"{labels[start + i]}: vicino a una singolarità "
                    f"(sigma_min del Jacobiano = {sigma_min[i]:.4f} < {self.limits.min_sigma:.4f})"
                )
                if self.violation_count >= _MAX_REPORTED_VIOLATIONS:
                    return

    def velocity(self, ratio: float, where: str) -> None:
        self.max_velocity_ratio = max(self.max_velocity_ratio, ratio)
        if ratio > self.limits.velocity_fraction:
            self.fail(
                f"{where}: velocità di picco {ratio * 100:.0f}% del limite di giunto "
                f"(massimo consentito {self.limits.velocity_fraction * 100:.0f}%)"
            )

    def acceleration(self, ratio: float, where: str) -> None:
        self.max_acceleration_ratio = max(self.max_acceleration_ratio, ratio)
        if ratio > self.limits.acceleration_fraction:
            self.fail(
                f"{where}: accelerazione di picco {ratio * 100:.0f}% del limite di giunto "
                f"(massimo consentito {self.limits.acceleration_fraction * 100:.0f}%)"
            )

    def report(self) -> Dict[str, Any]:
        return {
            "ok": self.violation_count == 0,
            "violation_count": self.violation_count,
            "violations": list(self.violations),
            "min_sigma": self.min_sigma,
            "min_sigma_at": self.min_sigma_where,
            "max_velocity_ratio": round(self.max_velocity_ratio, 4),
            "max_acceleration_ratio": round(self.max_acceleration_ratio, 4),
            "num_samples": self.num_samples,
            "elapsed_ms": round((time.monotonic() - self.started) * 1000.0, 2),
        }

    def finish(self, what: str) -> Dict[str, Any]:
        report = self.report()
        if self.violation_count:
            extra = self.violation_count - len(self.violations)
            detail = "; ".join(self.violations)
            if extra > 0:
                detail += f"; (+{extra} altre violazioni)"
            raise TrajectoryValidationError(
                f"Controllo pre-esecuzione fallito per {what}: {detail}", report
            )
        return report


def validate_stop_and_go(
    q_start: Sequence[float],
    waypoints: Sequence[Sequence[float]],
    segment_durations: Sequence[float],
    limits: Optional[ValidationLimits] = None,
) -> Dict[str, Any]:
    """
    Verifica una traiettoria eseguita punto per punto (ogni segmento è un
    movimento minimum-jerk fermo→fermo di durata segment_durations[k]).
    Solleva TrajectoryValidationError se non è sicura.
    """
    limits = limits or ValidationLimits.from_env()
    check = _Check(limits)

    q_prev = np.asarray(q_start, dtype=float)
    points = np.asarray(waypoints, dtype=float)
    durations = np.asarray(segment_durations, dtype=float)
    if points.ndim != 2 or points.shape[1] != 7 or q_prev.shape != (7,):
        raise TrajectoryValidationError("Forma dei waypoint non valida (attesi 7 giunti).")
    if durations.shape != (points.shape[0],):
        raise ValueError("segment_durations deve avere un valore per waypoint.")

    all_q = [q_prev]
    labels: List[str] = ["stato corrente"]
    sample_q: List[np.ndarray] = []
    sample_labels: List[str] = []
    s = np.linspace(0.0, 1.0, max(2, limits.samples_per_segment) + 1)[1:, None]

    prev = q_prev
    for k, (wp, T) in enumerate(zip(points, durations)):
        delta = np.abs(wp - prev)
        check.velocity(
            float(np.max(MIN_JERK_PEAK_VELOCITY * delta / T / FR3_DQ_MAX)),
            f"segmento {k + 1}",
        )
        check.acceleration(
            float(np.max(MIN_JERK_PEAK_ACCELERATION * delta / T**2 / FR3_DDQ_MAX)),
            f"segmento {k + 1}",
        )
        sample_q.append(prev + s * (wp - prev))
        sample_labels.extend(f"segmento {k + 1} (s={v:.2f})" for v in s[:, 0])
        all_q.append(wp)
        labels.append(f"waypoint {k + 1}")
        prev = wp

    Q_way = np.asarray(all_q)
    Q_samples = np.concatenate(sample_q, axis=0)
    check.num_samples = Q_samples.shape[0] + 1

    check.joint_limits(Q_way, labels)
    check.check_budget()
    check.singularity(np.vstack([Q_way[:1], Q_samples]), ["stato corrente"] + sample_labels)
    return check.finish("traiettoria punto-punto")


def validate_sampled_trajectory(
    times: np.ndarray,
    Q: np.ndarray,
    limits: Optional[ValidationLimits] = None,
    what: str = "traiettoria fly",
) -> Dict[str, Any]:
    """
    Verifica una traiettoria già campionata nel tempo (es. fly motion):
    limiti di giunto, velocità (differenze finite) e singolarità lungo tutto
    il percorso. Solleva TrajectoryValidationError se non è sicura.
    """
    limits = limits or ValidationLimits.from_env()
    check = _Check(limits)
    times = np.asarray(times, dtype=float)
    Q = np.asarray(Q, dtype=float)
    check.num_samples = Q.shape[0]
    labels = [f"t={t:.2f}s" for t in times]

    dt = np.diff(times)
    if np.any(dt <= 0.0):
        raise TrajectoryValidationError("Campioni temporali non strettamente crescenti.")
    velocity = np.abs(np.diff(Q, axis=0)) / dt[:, None]
    check.velocity(float(np.max(velocity / FR3_DQ_MAX)), "percorso fly")

    check.joint_limits(Q, labels)
    check.singularity(Q, labels)
    return check.finish(what)
