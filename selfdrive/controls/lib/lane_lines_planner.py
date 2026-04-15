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

from collections import deque

import numpy as np


DEFAULT_LANE_WIDTH = 3.7          # m, US interstate standard; seed value
MIN_PROB_BOTH = 0.5               # threshold when both lanes are in play
MIN_PROB_SINGLE = 0.85            # very strict for single-side anchor
SINGLE_SIDE_CONF_DERATE = 0.2     # single-side conf caps around 0.2 -> blend caps
                                  # at ~0.4 (with conf/0.5 remap). Model dominates.
SINGLE_SIDE_OFFSET_SANITY_M = 0.6 # single-side commanded offset > 0.6 m rejects
                                  # (real lanes keep you within ~0.3 m of center)

# Rolling lane-width history. Updated only when both lanes are confidently detected.
# Gives a data-driven width for single-side fallback instead of assuming 3.7 m.
# modelV2 fires at 20 Hz, so 45 s = 900 samples.
WIDTH_HISTORY_SECONDS = 45.0
WIDTH_HISTORY_RATE_HZ = 20        # modelV2 publish rate
WIDTH_HISTORY_MAXLEN = int(WIDTH_HISTORY_SECONDS * WIDTH_HISTORY_RATE_HZ)
WIDTH_HISTORY_ACCEPT_PROB = 0.6   # both lanes must exceed this to trust the sample
WIDTH_HISTORY_SAMPLE_MIN_M = 2.7  # reject obvious bad detections (narrower than US lane)
WIDTH_HISTORY_SAMPLE_MAX_M = 4.5  # reject obvious bad detections (wider than US lane)
WIDTH_HISTORY_MIN_SAMPLES = 40    # need at least ~2 s of data before trusting history
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
# Pursuit: compromise between 1.2 s (weak centering) and 0.6 s (over-aggressive,
# caused on-road swerves when combined with runaway yaw-rate feedback). 0.8 s with
# 10-22 m bounds gives reasonable authority without the quadratic noise amplification
# of very short pursuit.
PURSUIT_SECONDS = 0.8
PURSUIT_MIN_M = 10.0
PURSUIT_MAX_M = 22.0

# Sanity cap on output. Vehicle can't physically curve tighter than this at normal
# speeds anyway, and producing larger values from noisy lane detections is pure harm.
CURVATURE_OUTPUT_CLIP = 0.015

# Above this speed, blinker + steering wheel motion means the model is running a
# lane change - the planner must step out of the way or it will fight the maneuver.
LANE_CHANGE_SPEED_MIN = 6.7       # m/s (~15 mph)


class LaneLinesPlanner:
  def __init__(self):
    self.confidence = 0.0
    self.last_curvature = 0.0
    self.width_history: deque[float] = deque(maxlen=WIDTH_HISTORY_MAXLEN)

  def _estimated_lane_width(self) -> float:
    if len(self.width_history) >= WIDTH_HISTORY_MIN_SAMPLES:
      return float(np.mean(self.width_history))
    return DEFAULT_LANE_WIDTH

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

    # Update width history when both lanes are solidly detected.
    if left_p > WIDTH_HISTORY_ACCEPT_PROB and right_p > WIDTH_HISTORY_ACCEPT_PROB:
      w0 = float(y_right[0] - y_left[0])
      if WIDTH_HISTORY_SAMPLE_MIN_M < w0 < WIDTH_HISTORY_SAMPLE_MAX_M:
        self.width_history.append(w0)

    lane_width = self._estimated_lane_width()

    # Both lanes: direct midpoint, full confidence.
    # Single-side: anchor using the history-averaged width (falls back to default
    # before enough samples accumulate). Offset sanity reject still in effect.
    if left_p > MIN_PROB_BOTH and right_p > MIN_PROB_BOTH:
      y_mid = 0.5 * (y_left + y_right)
      raw_conf = min(left_p, right_p)
    elif left_p > MIN_PROB_SINGLE:
      y_mid = y_left + 0.5 * lane_width
      if abs(float(y_mid[0])) > SINGLE_SIDE_OFFSET_SANITY_M:
        self.confidence *= CONF_DECAY_PER_FRAME
        return self.last_curvature, self.confidence
      raw_conf = left_p * SINGLE_SIDE_CONF_DERATE
    elif right_p > MIN_PROB_SINGLE:
      y_mid = y_right - 0.5 * lane_width
      if abs(float(y_mid[0])) > SINGLE_SIDE_OFFSET_SANITY_M:
        self.confidence *= CONF_DECAY_PER_FRAME
        return self.last_curvature, self.confidence
      raw_conf = right_p * SINGLE_SIDE_CONF_DERATE
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
