"""
Fly motion (fly-by / continuous path) nello spazio dei giunti.

Una traiettoria a N punti viene eseguita senza fermarsi nei punti intermedi:
ogni vertice interno è sostituito da una "zona di raccordo" (corner zone) in
cui il percorso curva dolcemente invece di raggiungere il vertice. La velocità
lungo il percorso è continua: rampa di accelerazione smooth all'inizio, tratto
a velocità costante, rampa di decelerazione smooth alla fine.

Modello:
  - percorso geometrico: spezzata q0→q1→…→qN dove ogni vertice interno è
    arrotondato con una Bézier quadratica (continuità di tangente, C1). Il
    raggio della zona di raccordo è `blend_radius` (rad, norma euclidea nello
    spazio dei giunti), limitato al 50% dei due segmenti adiacenti;
  - legge temporale: s(t) con rampe di velocità a polinomio quintico
    (accelerazione continua) e velocità di crociera v_c ricavata da
    velocità/accelerazione massime dei giunti, curvatura dei raccordi e
    speed_factor / durata richiesta;
  - inversioni di direzione quasi complete (>~162°) non possono essere
    "volate": il percorso viene spezzato e il robot si ferma in quel vertice.

Il modulo è solo numpy: nessuna dipendenza da pylibfranka.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .kinematics import FR3_DQ_MAX
from .trajectory_validation import FR3_DDQ_MAX, ValidationLimits

# Margine di progetto rispetto ai limiti del pre-check: il controllo finale
# usa differenze finite, quindi la traiettoria viene generata un po' sotto soglia.
_DESIGN_MARGIN = 0.95
_MIN_VERTEX_DISTANCE = 1e-6
_REVERSAL_COS = -0.95
_RAMP_PEAK = 1.875  # picco di w'(τ) per w(τ) = 10τ³ - 15τ⁴ + 6τ⁵


def _ramp_integral(tau: float) -> float:
    """∫₀^τ w(u) du con w(u) = 10u³ - 15u⁴ + 6u⁵."""
    return 2.5 * tau**4 - 3.0 * tau**5 + tau**6


def _dedupe_vertices(vertices: Sequence[np.ndarray]) -> List[np.ndarray]:
    out = [np.asarray(vertices[0], dtype=float)]
    for v in vertices[1:]:
        v = np.asarray(v, dtype=float)
        if np.linalg.norm(v - out[-1]) > _MIN_VERTEX_DISTANCE:
            out.append(v)
    return out


def split_at_reversals(vertices: Sequence[np.ndarray]) -> List[List[np.ndarray]]:
    """Spezza la lista di vertici dove la direzione si inverte quasi del tutto."""
    parts: List[List[np.ndarray]] = [[vertices[0]]]
    for i in range(1, len(vertices) - 1):
        parts[-1].append(vertices[i])
        d_in = vertices[i] - vertices[i - 1]
        d_out = vertices[i + 1] - vertices[i]
        cos = float(np.dot(d_in, d_out) / (np.linalg.norm(d_in) * np.linalg.norm(d_out)))
        if cos < _REVERSAL_COS:
            parts.append([vertices[i]])
    parts[-1].append(vertices[-1])
    return parts


def build_blended_path(
    vertices: Sequence[np.ndarray],
    blend_radius: float,
    points_per_blend: int = 24,
) -> np.ndarray:
    """Spezzata con vertici interni raccordati da Bézier quadratiche. Ritorna (M, 7)."""
    V = [np.asarray(v, dtype=float) for v in vertices]
    pts: List[np.ndarray] = [V[0]]
    taus = np.linspace(0.0, 1.0, points_per_blend)[:, None]
    for i in range(1, len(V) - 1):
        d_in, d_out = V[i] - V[i - 1], V[i + 1] - V[i]
        l_in, l_out = np.linalg.norm(d_in), np.linalg.norm(d_out)
        u_in, u_out = d_in / l_in, d_out / l_out
        radius = min(blend_radius, 0.5 * l_in, 0.5 * l_out)
        if radius < _MIN_VERTEX_DISTANCE or float(np.dot(u_in, u_out)) > 0.9999:
            pts.append(V[i])
            continue
        a, b = V[i] - u_in * radius, V[i] + u_out * radius
        curve = (1 - taus) ** 2 * a + 2 * taus * (1 - taus) * V[i] + taus**2 * b
        pts.extend(curve)
    pts.append(V[-1])
    return np.asarray(pts)


@dataclass
class _Part:
    table: np.ndarray       # (M, 7) punti del percorso
    s_table: np.ndarray     # (M,) ascissa curvilinea [rad]
    length: float
    v_cruise: float
    ramp_time: float
    duration: float
    limited_by: str

    def s_of_t(self, t: float) -> float:
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.length
        v, tr = self.v_cruise, self.ramp_time
        if t < tr:
            return v * tr * _ramp_integral(t / tr)
        if t <= self.duration - tr:
            return v * tr * 0.5 + v * (t - tr)
        return self.length - v * tr * _ramp_integral((self.duration - t) / tr)

    def q_at(self, t: float) -> np.ndarray:
        s = self.s_of_t(t)
        i = int(np.clip(np.searchsorted(self.s_table, s, side="right") - 1, 0, len(self.s_table) - 2))
        span = self.s_table[i + 1] - self.s_table[i]
        f = 0.0 if span <= 0.0 else (s - self.s_table[i]) / span
        return self.table[i] + f * (self.table[i + 1] - self.table[i])


@dataclass
class FlyTrajectory:
    parts: List[_Part] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return float(sum(p.duration for p in self.parts))

    @property
    def end_point(self) -> Optional[np.ndarray]:
        return self.parts[-1].table[-1] if self.parts else None

    def q_at(self, t: float) -> np.ndarray:
        if not self.parts:
            raise ValueError("traiettoria fly vuota")
        for part in self.parts:
            if t <= part.duration:
                return part.q_at(t)
            t -= part.duration
        return self.parts[-1].table[-1].copy()

    def sample(self, dt: float = 0.02):
        """Campiona (times, Q) a passo dt, includendo sempre l'istante finale."""
        total = self.duration
        times = np.arange(0.0, total, dt)
        if times.size == 0 or times[-1] < total:
            times = np.append(times, total)
        return times, np.array([self.q_at(float(t)) for t in times])

    def summary(self) -> Dict[str, Any]:
        return {
            "duration": round(self.duration, 3),
            "num_parts": len(self.parts),
            "length_rad": round(float(sum(p.length for p in self.parts)), 4),
            "cruise_speed_rad_s": [round(p.v_cruise, 4) for p in self.parts],
            "limited_by": [p.limited_by for p in self.parts],
        }


def _plan_part(
    vertices: Sequence[np.ndarray],
    blend_radius: float,
    speed_factor: float,
    requested_duration: Optional[float],
    limits: ValidationLimits,
    ramp_fraction: float,
) -> _Part:
    table = build_blended_path(vertices, blend_radius)
    steps = np.diff(table, axis=0)
    ds = np.linalg.norm(steps, axis=1)
    keep = ds > 1e-9
    table = np.vstack([table[:1], table[1:][keep]])
    steps, ds = steps[keep], ds[keep]
    s_table = np.concatenate([[0.0], np.cumsum(ds)])
    length = float(s_table[-1])

    tangents = np.abs(steps / ds[:, None])  # |dq_j/ds| per intervallo
    eps = 1e-9
    v_frac = limits.velocity_fraction * _DESIGN_MARGIN
    a_frac = limits.acceleration_fraction * _DESIGN_MARGIN

    v_joint_limit = float(np.min(FR3_DQ_MAX[None, :] / np.maximum(tangents, eps)))
    a_tan = float(np.min(a_frac * FR3_DDQ_MAX[None, :] / np.maximum(tangents, eps)))
    v_vel_cap = v_frac * v_joint_limit

    unit = steps / ds[:, None]
    if len(unit) > 1:
        mid_ds = 0.5 * (ds[:-1] + ds[1:])
        curvature = np.abs(np.diff(unit, axis=0)) / mid_ds[:, None]
        c_max = np.max(curvature, axis=0)
        with np.errstate(divide="ignore"):
            v_curv_cap = float(np.sqrt(np.min(a_frac * FR3_DDQ_MAX / np.maximum(c_max, eps))))
    else:
        v_curv_cap = math.inf

    if requested_duration is not None:
        v_req = length / (requested_duration * (1.0 - ramp_fraction))
    else:
        v_req = speed_factor * v_joint_limit

    v_cruise, limited_by = v_req, "richiesta"
    if v_vel_cap < v_cruise:
        v_cruise, limited_by = v_vel_cap, "limite velocità giunti"
    if v_curv_cap < v_cruise:
        v_cruise, limited_by = v_curv_cap, "curvatura dei raccordi"

    ramp = max(ramp_fraction * length / (v_cruise * (1.0 - ramp_fraction)),
               _RAMP_PEAK * v_cruise / a_tan)
    if ramp > length / v_cruise:  # percorso corto: nessun tratto a velocità costante
        ramp = math.sqrt(_RAMP_PEAK * length / a_tan)
        v_cruise = length / ramp
        limited_by = "limite accelerazione giunti"
    duration = length / v_cruise + ramp

    return _Part(table, s_table, length, v_cruise, ramp, duration, limited_by)


def plan_fly_trajectory(
    q_start: Sequence[float],
    waypoints: Sequence[Sequence[float]],
    blend_radius: float = 0.15,
    speed_factor: float = 0.2,
    requested_duration: Optional[float] = None,
    limits: Optional[ValidationLimits] = None,
    ramp_fraction: float = 0.2,
) -> FlyTrajectory:
    """
    Costruisce la traiettoria fly attraverso i waypoint a partire da q_start.

    Args:
        blend_radius: raggio delle zone di raccordo [rad]; 0 = nessun raccordo.
        speed_factor: frazione (0,1] del limite di velocità dei giunti usata
            come velocità di crociera, se requested_duration è None.
        requested_duration: durata totale desiderata [s]. Se i limiti di
            velocità/accelerazione non la permettono, la durata viene
            allungata (mai accorciata): vedi FlyTrajectory.summary()["limited_by"].
    """
    if not (blend_radius >= 0.0 and math.isfinite(blend_radius)):
        raise ValueError("blend_radius deve essere finito e >= 0")
    if not (0.0 < speed_factor <= 1.0):
        raise ValueError("speed_factor deve essere in (0, 1]")
    if requested_duration is not None and not (requested_duration > 0.0 and math.isfinite(requested_duration)):
        raise ValueError("durata richiesta non valida")
    if not (0.0 < ramp_fraction < 0.5):
        raise ValueError("ramp_fraction deve essere in (0, 0.5)")
    limits = limits or ValidationLimits.from_env()

    vertices = _dedupe_vertices([np.asarray(q_start, dtype=float), *map(np.asarray, waypoints)])
    if len(vertices) < 2:
        return FlyTrajectory()

    parts = split_at_reversals(vertices)
    total_length = sum(
        float(np.sum(np.linalg.norm(np.diff(np.asarray(p), axis=0), axis=1))) for p in parts
    )
    parts_out = []
    for p in parts:
        part_len = float(np.sum(np.linalg.norm(np.diff(np.asarray(p), axis=0), axis=1)))
        part_duration = (
            None if requested_duration is None
            else max(requested_duration * part_len / total_length, 1e-3)
        )
        parts_out.append(
            _plan_part(p, blend_radius, speed_factor, part_duration, limits, ramp_fraction)
        )
    return FlyTrajectory(parts_out)
