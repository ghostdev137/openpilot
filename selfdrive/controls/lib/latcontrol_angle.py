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
    # ford-lka sim: close the loop around the open-loop kinematic controller. Adds two
    # feedback terms so disturbances (crosswind) don't have to propagate through the vision
    # pipeline before being rejected.
    self.ford_closed_loop = CP.brand == "ford"
    self.ford_kp_angle = 0.5     # amplification on angle tracking error
    self.ford_kp_yawrate = 1.0   # gain on equivalent-angle derived from yaw-rate error

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
        # snapshot target before mutating, so both errors are against the planner's demand
        target = angle_steers_des
        angle_err_deg = target - CS.steeringAngleDeg
        # desired yaw rate from curvature identity; convert (desired - actual) yaw-rate
        # error into the steering-wheel angle that would produce it at current speed
        desired_yaw_rate = -desired_curvature * CS.vEgo
        yaw_rate_err = desired_yaw_rate - CS.yawRate
        yaw_err_angle_deg = math.degrees(yaw_rate_err * VM.l / max(CS.vEgo, 1.0)) * VM.sR
        angle_steers_des += self.ford_kp_angle * angle_err_deg
        angle_steers_des += self.ford_kp_yawrate * yaw_err_angle_deg

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
