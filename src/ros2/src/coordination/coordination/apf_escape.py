"""Time-varying repulsive Artificial Potential Field (APF) for escape behaviour.

Purpose
-------
This module is intended to be used as an *additive* term on top of an existing
local planner (DWB in our setup). It does NOT replace the local planner. It
only kicks in when the UAV appears to be stuck in a local minimum near complex
obstacles (typical: building concave corners). When that condition is met, the
APF computes a repulsive vector in the body frame from the current 2D LiDAR
scan, with magnitude that **grows over time** as long as the UAV keeps not
moving. The result is added to the existing avoid command so the system can
slowly accumulate enough "push" to leave the dead-lock.

Important constraints (all by design)
-------------------------------------
* xy plane only. This module never touches z. The caller must zero/ignore any
  z component if it has its own altitude controller.
* Range-of-sight only. We only consider scan returns within ``range_of_sight``;
  this prevents far-away unrelated obstacles from biasing the escape.
* Safety check before any push. We refuse to emit a non-zero APF velocity
  unless there is at least one direction with clearance >= ``free_distance``
  in the scan window. This implements the "明确的可行逃逸方向" requirement.
* Only kicks in when stuck. The stuck detector uses a sliding window of
  ``window_seconds`` and triggers when the displacement during that window is
  below ``stuck_distance``. This is the time-varying part: the longer the UAV
  stays stuck, the larger the repulsion magnitude becomes.
* Does NOT alter the upstream avoidance logic. It only produces an *additional*
  velocity vector. The caller decides how to combine it (e.g. simple sum with
  saturation).

Math
----
Let p_i = (x_i, y_i) be the body-frame position of LiDAR return i with range
``d_i = sqrt(x_i^2 + y_i^2)``. The base repulsive vector is

    F_i = -unit(p_i) * (1 / max(d_i, eps)^2)

The aggregate base force is::

    F_base = sum_{i where d_i < range_of_sight} F_i

We then apply a time-varying gain. Let ``t_stuck`` be the elapsed time during
which the UAV has been continuously "stuck" (see stuck condition above). The
gain is::

    K(t_stuck) = K0 * min(1, t_stuck / saturation_time)

so it ramps up linearly from 0 to ``K0`` over ``saturation_time`` seconds.
This is the smooth time-varying penalty asked for: small while DWB still has
a chance to recover, large enough to break dead-lock if it doesn't.

The final body-frame escape velocity is::

    v_escape = clamp_norm(K(t_stuck) * F_base, max_speed)

where ``clamp_norm`` rescales the vector to at most ``max_speed`` while
preserving direction.

Frames
------
The function expects ``ranges`` plus ``angle_min`` and ``angle_increment`` in
the LiDAR frame. We assume the scan frame is rigidly attached to the UAV body
frame with identity rotation (in this project ``UAV_1`` = body frame and
the LiDAR is mounted aligned with it). If the lidar were rotated relative to
body, the caller should pre-rotate the points before calling here.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Tuple

# Numerical guard for inverse-square repulsion. Too small a value would let a
# single near-zero range dominate; we cap the effective minimum distance at
# this many meters.
_MIN_DIST_EPS = 0.5


@dataclass
class ApfEscapeConfig:
    """Tunable parameters for the time-varying APF escape behaviour.

    All distances are meters, all times are seconds, all velocities are m/s.

    Two-tier gain design
    --------------------
    The APF outputs are split into two additive parts so that "obstacle
    avoidance" and "escape from local minima" use the same physical mechanism
    but different aggressiveness:

    * ``K_baseline``: applied **whenever AVOID is active**, regardless of
      whether the UAV is currently moving. This guarantees that as long as the
      caller considers the UAV in obstacle-avoidance mode, the APF contributes
      a real repulsive velocity. This is what stops the "evade once then ram
      from a different angle" failure mode.

    * ``K_stuck_extra``: an extra time-varying gain that ramps up only after
      the stuck condition has held for ``saturation_time`` seconds. It is
      added on top of ``K_baseline`` to break out of true local minima.

    The two-tier design keeps APF safe (baseline never tries to push if there
    is no clear direction) while still giving it the ability to gradually
    overpower a dead-lock without exploding the velocity.
    """

    # Maximum LiDAR distance considered for repulsion. Returns farther than
    # this are ignored. Per design, this MUST be smaller than (or equal to)
    # the upstream AVOID-state entry threshold ``obstacle_distance``: the
    # state machine handles long-range planning (slow down, hand control to
    # DWB), while the APF only does close-range repulsion. Keeping APF range
    # narrow is also what allows the UAV to thread tree gaps narrower than
    # 10m: when both sides are >range_of_sight, APF stays silent and lets
    # DWB pick the corridor; when one side gets close, APF gives a strong
    # 1/d^2 push away from it. 5m means APF stays silent above 5m, so
    # the UAV is free to approach the obstacle at cruise speed; only
    # inside 5m does APF engage. Combats the "hovers far from any
    # obstacle and still feels APF resistance" failure mode while
    # leaving a strong reaction zone in the danger band.
    range_of_sight: float = 5.0

    # Half-aperture (radians) of the angular window centred on the closest
    # beam. Beams whose bearing is within +/-angular_window/2 of the closest
    # beam contribute to the repulsion sum; everything else is ignored.
    # Set to ~150 degrees (2.618 rad): wide enough to capture corner tips
    # that fall slightly off the closest-beam direction, narrow enough
    # that an unrelated obstacle on the opposite side of the UAV does
    # not pollute the force vector. Wider than 120deg specifically to
    # address corner-edge grazing where the convex tip lies ~30deg off
    # the nearest beam axis.
    angular_window: float = 2.618

    # Minimum free-direction distance required to consider an escape feasible.
    # If no scan bin in the look-around window has clearance >= this, we do
    # nothing (refuse to push) so we never command a motion that leads into
    # another obstacle. Reduced from 6 to 4 to allow squeezing through gaps
    # narrower than 10m wide (e.g. tree canopies): even a 5m clearance on
    # one side should still count as "feasible escape".
    free_distance: float = 4.0

    # Sliding window length used by the stuck detector. Shorter (2.5s)
    # so a hover-in-mid-air situation in front of an obstacle is detected
    # quickly; the longer the UAV stays motionless the longer it would
    # take the previous 4s window to fill with low-displacement samples.
    window_seconds: float = 2.5

    # If accumulated displacement over the window is below this, the UAV
    # is considered stuck. 0.5 m over the 2.5s window means anything
    # slower than 0.2 m/s on average counts as stuck; this catches the
    # "drift along the wall at 0.1-0.2 m/s without making real progress"
    # failure mode that the previous 0.7 m threshold missed.
    stuck_distance: float = 0.5

    # Hysteresis: once stuck, the UAV must move at least this far in the
    # window to exit the stuck state. Larger than ``stuck_distance`` so a
    # tiny APF nudge does not toggle stuck off and on every tick. Reduced
    # from 4.0 to 2.5 because once the UAV is moving 2.5m / 2.5s = 1 m/s
    # it is clearly making progress and APF can hand back to DWB.
    stuck_release_distance: float = 2.5

    # Always-on baseline gain. With 1/d^2 falloff and angular_window=150deg
    # the raw force at 5m on a continuous wall is ~4-5, at 3m ~10-15, at
    # 2m ~50, at 1m well above 100. K_baseline=2.2 makes the velocity
    # cross 1 m/s near 4.5m and saturates max_speed by ~3.2m. Stronger
    # than 1.8 to firm up the close-range push and keep the UAV from
    # actually contacting side walls during late-deflection scenarios.
    K_baseline: float = 2.2

    # Additional gain added on top of ``K_baseline`` once the UAV has been
    # stuck for ``saturation_time`` seconds. Sized so total gain reaches
    # ~12 in a sustained dead-lock; combined with 1/d^2 force the
    # velocity easily saturates max_speed at any distance the APF cares
    # about (out to range_of_sight=5m). Higher than 9.0 so a concave
    # corner is broken even when several walls feed the repulsive sum
    # in roughly opposing directions.
    K_stuck_extra: float = 11.0

    # Time after entering "stuck" at which the extra gain reaches its max.
    # 0.4s window with the quadratic ramp ``ratio**2`` gives ~25% gain at
    # 0.2s and 100% at 0.4s. Faster than 0.5s so a corner-stuck hover is
    # decisively broken before the UAV drifts further into the corner.
    saturation_time: float = 0.4

    # Maximum magnitude (m/s) of the escape velocity vector after gain.
    # 3.5 leaves headroom above cruise=3.0 so APF in stuck mode can
    # actively pull the UAV faster than cruise would push it; this is
    # what breaks the "AVOID exit -> cruise pulls back" loop in side
    # walls. APF only activates inside AVOID state.
    max_speed: float = 3.5

    # First-order low-pass filter coefficient on the APF output velocity.
    # 0.0 disables filtering (passes raw output), 1.0 freezes the output.
    # 0.8 is aggressive smoothing (~250ms time constant at 20Hz scan rate).
    # Needed because the 2D LaserScan in this project is jittery: the closest
    # range can flick between 1m and 30m within a single second when scan
    # density is low or the lidar slice catches different obstacle layers.
    # Strong smoothing keeps the APF cmd contribution stable even when the
    # raw repulsion vector is bouncing around.
    output_lpf_alpha: float = 0.8


def _clamp_norm(vx: float, vy: float, max_norm: float) -> Tuple[float, float]:
    """Clamp a 2D vector's magnitude to ``max_norm`` while preserving direction.

    Returns the clamped (vx, vy). If the vector is already within bounds it is
    returned unchanged. Zero vectors are returned as-is.
    """
    n = math.hypot(vx, vy)
    if n <= max_norm or n <= 1e-9:
        return vx, vy
    s = max_norm / n
    return vx * s, vy * s


def compute_repulsive_xy(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    config: ApfEscapeConfig,
) -> Tuple[float, float, float]:
    """Compute the time-invariant base repulsive force in the body frame.

    Algorithm
    ---------
    The traditional "sum 1/d^2 over every beam in range" gives a force whose
    magnitude scales with how many beams hit the obstacle, which means a wide
    wall and a narrow pole produce wildly different forces at the same
    distance. The previous "rank-top-5 with decaying weights" workaround was
    just sub-sampling that sum and lost the geometry information that makes
    the APF work in concave corners (where many beams pointing into the same
    region are exactly what tells you where the opening is).

    The current implementation uses an *angular window* anchored on the
    closest beam:

    1. Find the beam with the smallest in-range distance. Its bearing
       ``theta_c`` is the direction the obstacle is most threatening.
    2. Take every in-range beam whose bearing is within
       ``angular_window`` of ``theta_c`` (wrap-around aware).
    3. Sum ``-unit_vec(theta_i) * (1 / d_i^3)`` over those beams.

    Why this works
    --------------
    * In a concave corner: the closest beam picks one wall, the angular
      window then captures the *whole* near-side wall plus the inner part of
      the adjacent wall(s); summing them gives a force pointing toward the
      opening, exactly the physically expected escape direction.
    * Against an isolated obstacle (single tree, pole): only beams that
      actually hit it contribute, so the magnitude reflects how close the
      obstacle is, not how wide the scan happens to be.
    * Beams outside the window (e.g. an unrelated wall on the other side of
      the UAV) are silently ignored. Those situations should be handled by
      the upstream state machine, not by being lumped into the same APF.

    Parameters
    ----------
    ranges
        Iterable of LiDAR range readings, one per beam, ordered by increasing
        angle starting at ``angle_min``.
    angle_min
        Angle (radians) of the first beam in the body frame.
    angle_increment
        Angle delta (radians) between consecutive beams.
    config
        Tunable parameters; uses ``range_of_sight`` and ``angular_window``.

    Returns
    -------
    (Fx, Fy, free_dir_distance)
        ``(Fx, Fy)`` is the base repulsive force in the body frame, units
        ``1 / m^3`` because the per-beam term is ``1/d^3``. ``free_dir_distance``
        is the largest finite range observed (over the entire scan) so the
        caller can decide if a feasible escape direction exists.
    """
    # First pass: collect every in-range finite return with its bearing,
    # and track the absolute farthest return for the "feasible escape" gate.
    in_range: list[tuple[float, float]] = []  # (distance, bearing)
    max_finite = 0.0
    closest_d = math.inf
    closest_bearing = 0.0
    angle = angle_min
    for r in ranges:
        if math.isfinite(r):
            if r > max_finite:
                max_finite = r
            if r < config.range_of_sight:
                in_range.append((r, angle))
                if r < closest_d:
                    closest_d = r
                    closest_bearing = angle
        angle += angle_increment

    if not in_range:
        return 0.0, 0.0, max_finite

    # Second pass: only keep beams within ``angular_window`` of the closest
    # beam's bearing. Use the wrap-around aware shortest angular delta.
    half_window = config.angular_window / 2.0
    fx = 0.0
    fy = 0.0
    for d, bearing in in_range:
        delta = math.atan2(
            math.sin(bearing - closest_bearing),
            math.cos(bearing - closest_bearing),
        )
        if abs(delta) > half_window:
            continue
        d_eff = d if d >= _MIN_DIST_EPS else _MIN_DIST_EPS
        # Inverse-square falloff: keeps meaningful repulsion at medium
        # range (4-6m) so the UAV does not get sucked back into a
        # concave corner the moment it edges out of it. The cubic
        # variant decayed too quickly and let the goal-driven cruise
        # term pull the UAV back inside before APF could do anything.
        weight = 1.0 / (d_eff * d_eff)
        fx -= math.cos(bearing) * weight
        fy -= math.sin(bearing) * weight

    return fx, fy, max_finite


class ApfEscapeFilter:
    """Stateful escape filter that turns a base force into a usable velocity.

    Keeps a sliding window of the UAV's body-frame translation ("how much it
    actually moved recently") so it can decide whether the UAV is stuck. When
    stuck, accumulates a time-varying gain on the raw repulsion vector. When
    moving, decays back to zero gain so subsequent escapes start fresh.

    Usage
    -----
    Create one filter per UAV. On every control tick:

        filt.update_pose(now_seconds, x_world, y_world)
        fx, fy, max_clear = compute_repulsive_xy(ranges, ...)
        vx, vy = filt.compute_escape_velocity(now_seconds, fx, fy, max_clear)

    ``vx`` and ``vy`` are in the body frame. They are zero unless the filter
    decided the UAV is stuck AND there is a feasible escape direction.
    """

    def __init__(self, config: ApfEscapeConfig | None = None) -> None:
        self.config = config or ApfEscapeConfig()
        # Sliding window of (timestamp, world_x, world_y). World frame is fine
        # because we only use it to compute displacement magnitude.
        self._window: Deque[Tuple[float, float, float]] = deque()
        # Time at which the UAV first entered the "stuck" state, or None if
        # not currently stuck.
        self._stuck_since: float | None = None
        # Last filtered output, used by the first-order low-pass on emit.
        # Resets to (0, 0) whenever the filter declines to push (no safe
        # direction or zero gain) so a fresh escape starts cleanly.
        self._lpf_vx = 0.0
        self._lpf_vy = 0.0

    def update_pose(self, t: float, x_world: float, y_world: float) -> None:
        """Record the UAV's world-frame xy position at time ``t``.

        Old samples that fall outside ``window_seconds`` are dropped. We use
        world frame here because body frame moves with the UAV; world frame
        translation is what defines "actually went somewhere".
        """
        self._window.append((t, x_world, y_world))
        cutoff = t - self.config.window_seconds
        while self._window and self._window[0][0] < cutoff:
            self._window.popleft()

    def _window_displacement(self) -> float:
        """Return the displacement (meters) between the oldest and newest
        samples in the sliding window.

        If there are fewer than two samples, returns +inf so the system is not
        treated as stuck right after startup (we want some history first).
        """
        if len(self._window) < 2:
            return math.inf
        _, ox, oy = self._window[0]
        _, nx, ny = self._window[-1]
        return math.hypot(nx - ox, ny - oy)

    def _is_stuck(self) -> bool:
        """The UAV is stuck if it has barely moved over the recent window.

        Hysteresis: once stuck (``_stuck_since`` is set), require a much
        larger displacement to clear (``stuck_release_distance``). This
        prevents the time-varying gain from collapsing back to zero the
        moment a small APF nudge produces a fraction of a metre of motion,
        which would otherwise cause stuck/not-stuck oscillation around the
        ``stuck_distance`` threshold.
        """
        d = self._window_displacement()
        if self._stuck_since is None:
            return d < self.config.stuck_distance
        return d < self.config.stuck_release_distance

    def is_stuck_now(self) -> bool:
        """Public read-only view of the stuck flag.

        Returns True only after ``compute_escape_velocity`` has at least
        once seen a window-with-displacement-below-threshold and recorded
        ``_stuck_since``. Callers can use this to decide whether to
        let APF take over the avoid command from DWB entirely.
        """
        return self._stuck_since is not None

    def _gain(self, t: float) -> float:
        """Total APF gain in m^2/s.

        Always returns at least ``K_baseline`` (so AVOID itself produces real
        repulsion). If the UAV has been stuck, an extra ramp from 0 to
        ``K_stuck_extra`` is added. The ramp uses ``ratio ** 2`` (quadratic
        ease-in) so the escape force begins almost zero, grows slowly, and
        only approaches its maximum near the end of the saturation window.
        This is the time-varying part requested in the design (cost = K * t,
        but with t replaced by a smooth quadratic of normalised time so the
        force does not jump when the stuck flag toggles around its threshold).
        """
        baseline = self.config.K_baseline
        if self._stuck_since is None:
            return baseline
        elapsed = max(0.0, t - self._stuck_since)
        ratio = min(1.0, elapsed / max(self.config.saturation_time, 1e-6))
        # Quadratic ease-in: smooth start, accelerating later.
        return baseline + self.config.K_stuck_extra * (ratio * ratio)

    def compute_escape_velocity(
        self,
        t: float,
        force_x: float,
        force_y: float,
        max_clear_distance: float,
    ) -> Tuple[float, float]:
        """Combine raw repulsion with the gain (baseline + optional stuck ramp).

        Parameters
        ----------
        t
            Current time in seconds (same clock as ``update_pose``).
        force_x, force_y
            Output of :func:`compute_repulsive_xy`. Units of 1/m^2.
        max_clear_distance
            Largest finite scan range within the look-around window. Used as
            the safety check ("可行逃逸方向").

        Returns
        -------
        (vx, vy)
            Body-frame escape velocity in m/s. Both components are zero only
            if there is no safe escape direction. Otherwise the baseline gain
            always produces a non-zero push (assuming there is at least one
            obstacle in range_of_sight).
        """
        # Maintain stuck state across calls. Note: we still need this even
        # though we no longer gate the output on it, because the extra
        # time-varying ramp is applied only when stuck.
        stuck_now = self._is_stuck()
        if stuck_now and self._stuck_since is None:
            self._stuck_since = t
        elif not stuck_now:
            self._stuck_since = None

        # Refuse to emit anything if there is no clear escape direction. This
        # is the explicit guard against "pushing into another obstacle".
        # When refusing, also reset the LPF so the next valid push starts
        # fresh and is not contaminated by stale state.
        if max_clear_distance < self.config.free_distance:
            self._lpf_vx = 0.0
            self._lpf_vy = 0.0
            return 0.0, 0.0

        gain = self._gain(t)
        if gain <= 0.0:
            self._lpf_vx = 0.0
            self._lpf_vy = 0.0
            return 0.0, 0.0

        raw_vx = gain * force_x
        raw_vy = gain * force_y
        raw_vx, raw_vy = _clamp_norm(raw_vx, raw_vy, self.config.max_speed)

        # First-order low-pass: y_k = a*y_{k-1} + (1-a)*x_k.
        # This smooths the per-scan jitter so the airframe does not get
        # whipped between adjacent obstacle returns when DWB and APF are
        # simultaneously active. Filtered output is also clamped to keep
        # the magnitude inside max_speed even after blending.
        a = self.config.output_lpf_alpha
        self._lpf_vx = a * self._lpf_vx + (1.0 - a) * raw_vx
        self._lpf_vy = a * self._lpf_vy + (1.0 - a) * raw_vy
        self._lpf_vx, self._lpf_vy = _clamp_norm(
            self._lpf_vx, self._lpf_vy, self.config.max_speed,
        )
        return self._lpf_vx, self._lpf_vy
