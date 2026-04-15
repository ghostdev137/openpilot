import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    # ford-lka sim: lightly close the loop around the open-loop kinematic controller.
    # Prior aggressive version (kp_angle=0.5, kp_yawrate=1.0) blew up on-road: yaw-rate
    # feedback commanded 70 deg steering with 0 damping from stripped PSCM rate limits.
    # Keep just a small angle-error boost. Yaw-rate feedback DISABLED until re-tuned
    # with proper anti-windup and output clamping.
    self.ford_closed_loop = CP.brand == "ford"
    self.ford_kp_angle = 0.15    # modest P on angle tracking error
    self.ford_kp_yawrate = 0.0   # disabled - caused runaway on-road
    # Hard sanity clip on final commanded wheel angle. Vehicle never needs more than
    # this for any realistic lane-keeping scenario on a straight road.
    self.ford_angle_clip_deg = 30.0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg
      if self.ford_closed_loop:
        target = angle_steers_des
        angle_err_deg = target - CS.steeringAngleDeg
        angle_steers_des += self.ford_kp_angle * angle_err_deg
        if self.ford_kp_yawrate > 0.0:
          desired_yaw_rate = -desired_curvature * CS.vEgo
          yaw_rate_err = desired_yaw_rate - CS.yawRate
          yaw_err_angle_deg = math.degrees(yaw_rate_err * VM.l / max(CS.vEgo, 1.0)) * VM.sR
          angle_steers_des += self.ford_kp_yawrate * yaw_err_angle_deg
        # hard sanity clip so no feedback math can command an unsafe wheel angle
        clip = self.ford_angle_clip_deg
        if angle_steers_des > clip: angle_steers_des = clip
        elif angle_steers_des < -clip: angle_steers_des = -clip

    if self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = angle_steers_des
    return 0, float(angle_steers_des), angle_log
