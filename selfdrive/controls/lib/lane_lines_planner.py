"""ford-lka sim: hybrid lane-lines-based curvature planner.

The supercombo end-to-end action.desiredCurvature is noisy and lazy on Transit LKA.
The model's upstream laneLines output (pure vision geometry) is cleaner. This module
derives a curvature command by fitting a polynomial to the midpoint of left+right
lane lines and applying a pure-pursuit target, falling back to a single-side lane
with EMA-tracked lane width when only one line is confident. Returns a confidence
scalar so controlsd can blend against the model's e2e output (or drop to it entirely
when lane lines are unreliable).

Rlog survey on Transit segments: ~38% of frames have both lanes >0.3 prob,
~69% have at least one. ~31% genuinely need fallback - hybrid is mandatory.
"""
from __future__ import annotations

import numpy as np


DEFAULT_LANE_WIDTH = 3.7          # m, US interstate standard
MIN_PROB = 0.3                    # below this, treat lane as absent
BOTH_PROB_FOR_WIDTH_UPDATE = 0.5  # only update width EMA when both are strong
WIDTH_SANITY_MIN = 2.5
WIDTH_SANITY_MAX = 5.0
WIDTH_EMA_ALPHA = 0.2
CONF_EMA_ALPHA = 0.3
CONF_DECAY_PER_FRAME = 0.95       # when lanes absent entirely
# Output EMA disabled: on-road test showed the van wasn't pulling back to center
# (std=0.15 m offset, 55% of engaged frames > 0.1 m off). We need authority, not
# smoothness. Keeping alpha=1.0 means no filtering on the output.
OUTPUT_EMA_ALPHA = 1.0

# Fit window is fixed: experimentally, speed-scaling the fit window hurts highway
# steady-cruise variance because distant lane points feed polyfit noise into c2.
# 30 m is the sweet spot for Transit on rlog.
FIT_HORIZON_M = 30.0
# Pursuit is speed-scaled. Classic rule is 1-1.5 s of travel, but that produced an
# under-damped "drift around the center" behavior on the real Transit (55% of
# engaged frames were > 0.1 m off). Shorter pursuit -> curvature = 2y/x0^2 scales
# up quadratically, so 0.6 s gives ~4x more authority than 1.2 s for the same offset.
# Noise goes up, user explicitly accepts it.
PURSUIT_SECONDS = 0.6
PURSUIT_MIN_M = 8.0
PURSUIT_MAX_M = 18.0

# Sanity cap on output. Vehicle can't physically curve tighter than this at normal
# speeds anyway, and producing larger values from noisy lane detections is pure harm.
CURVATURE_OUTPUT_CLIP = 0.015

# Above this speed, blinker + steering wheel motion means the model is running a
# lane change - the planner must step out of the way or it will fight the maneuver.
LANE_CHANGE_SPEED_MIN = 6.7       # m/s (~15 mph)


class LaneLinesPlanner:
  def __init__(self):
    self.lane_width = DEFAULT_LANE_WIDTH
    self.confidence = 0.0
    self.last_curvature = 0.0

  def update(self, model_v2, v_ego: float = 0.0, lane_change_active: bool = False) -> tuple[float, float]:
    # During a lane change, hand control back to the model. We hold last_curvature
    # but decay confidence fast so the blend goes to model inside ~20 frames.
    if lane_change_active and v_ego > LANE_CHANGE_SPEED_MIN:
      self.confidence *= CONF_DECAY_PER_FRAME
      return self.last_curvature, self.confidence

    if not model_v2.laneLines or len(model_v2.laneLineProbs) < 3:
      self.confidence *= CONF_DECAY_PER_FRAME
      return self.last_curvature, self.confidence

    probs = list(model_v2.laneLineProbs)
    left_p, right_p = probs[1], probs[2]
    left = model_v2.laneLines[1]
    right = model_v2.laneLines[2]

    xs = np.asarray(left.x, dtype=np.float32)
    y_left = np.asarray(left.y, dtype=np.float32)
    y_right = np.asarray(right.y, dtype=np.float32)

    if left_p > BOTH_PROB_FOR_WIDTH_UPDATE and right_p > BOTH_PROB_FOR_WIDTH_UPDATE:
      w0 = float(y_right[0] - y_left[0])
      if WIDTH_SANITY_MIN < w0 < WIDTH_SANITY_MAX:
        self.lane_width = WIDTH_EMA_ALPHA * w0 + (1.0 - WIDTH_EMA_ALPHA) * self.lane_width

    if left_p > MIN_PROB and right_p > MIN_PROB:
      y_mid = 0.5 * (y_left + y_right)
      raw_conf = min(left_p, right_p)
    elif left_p > MIN_PROB:
      y_mid = y_left + 0.5 * self.lane_width
      raw_conf = left_p * 0.7
    elif right_p > MIN_PROB:
      y_mid = y_right - 0.5 * self.lane_width
      raw_conf = right_p * 0.7
    else:
      self.confidence *= CONF_DECAY_PER_FRAME
      return self.last_curvature, self.confidence

    mask = xs <= FIT_HORIZON_M
    if int(mask.sum()) < 5:
      self.confidence *= CONF_DECAY_PER_FRAME
      return self.last_curvature, self.confidence

    coeffs = np.polyfit(xs[mask], y_mid[mask], 2)  # [c2, c1, c0]
    c2, c1, c0 = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])

    # Pure-pursuit target: curvature to reach (x0, y_mid(x0)) from vehicle origin,
    # heading 0. Small-angle: k ~ 2 * y_target / x0^2. Includes lane curvature (c2),
    # heading error (c1), and lateral offset (c0) by construction.
    x0 = max(PURSUIT_MIN_M, min(PURSUIT_MAX_M, v_ego * PURSUIT_SECONDS))
    y_at_x0 = c0 + c1 * x0 + c2 * x0 * x0
    curvature_raw = 2.0 * y_at_x0 / (x0 * x0)
    curvature_raw = max(-CURVATURE_OUTPUT_CLIP, min(CURVATURE_OUTPUT_CLIP, curvature_raw))
    curvature = OUTPUT_EMA_ALPHA * curvature_raw + (1.0 - OUTPUT_EMA_ALPHA) * self.last_curvature

    self.confidence = CONF_EMA_ALPHA * raw_conf + (1.0 - CONF_EMA_ALPHA) * self.confidence
    self.last_curvature = curvature
    return curvature, self.confidence
