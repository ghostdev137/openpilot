"""
Simulates a Ford vehicle (Bronco Sport / Transit) on CAN bus for openpilot.
Sends all required CAN messages so openpilot thinks a real Ford is connected.
Reads openpilot's sendcan to close the control loop.
"""
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState


class SimulatedCarFord:
  """Simulates a Ford vehicle to openpilot via CAN messages."""

  DBC = "ford_lincoln_base_pt"

  def __init__(self):
    self.packer = CANPacker(self.DBC)
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'selfdriveState', 'carParams'])
    self.params = Params()
    self.idx = 0

    # Simulated vehicle state
    self.speed_kph = 0.0
    self.steering_angle = 0.0
    self.cruise_speed_kph = 80.0
    self.cruise_engaged = False
    self.cruise_available = True
    self.gas_pressed = False
    self.brake_pressed = False
    self.sapp_state = 0  # 0=Closed, 1=Open, 2=Active, 3=Fault

    # Counter for BrakeSysFeatures
    self.brk_counter = 0
    self.yaw_counter = 0

  def send_panda_state(self):
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    ps = dat.pandaStates[0]
    ps.pandaType = 'cuatro'
    ps.ignitionLine = True
    ps.ignitionCan = True
    ps.controlsAllowed = True
    ps.safetyModel = 'ford'
    ps.harnessStatus = 'normal'
    self.pm.send('pandaStates', dat)

  def send_can_messages(self, simulator_state: SimulatorState = None):
    msg = []
    bus0 = 0  # powertrain bus
    bus2 = 2  # camera bus

    speed_kph = self.speed_kph
    if simulator_state and simulator_state.valid:
      speed_kph = simulator_state.speed * 3.6

    # Compute checksum for BrakeSysFeatures
    self.brk_counter = (self.brk_counter + 1) % 16
    brk_data_raw = int(speed_kph / 0.01) & 0xFFFF
    brk_byte0 = (brk_data_raw >> 8) & 0xFF
    brk_byte1 = brk_data_raw & 0xFF
    brk_qf = 3  # quality flag valid
    brk_cs = 0xFF - ((brk_byte0 + brk_byte1 + brk_qf + self.brk_counter) & 0xFF)

    # BrakeSysFeatures (0x415) — vehicle speed, counter, checksum
    msg.append(self.packer.make_can_msg("BrakeSysFeatures", bus0, {
      "Veh_V_ActlBrk": speed_kph,
      "VehVActlBrk_D_Qf": 3,
      "VehVActlBrk_No_Cnt": self.brk_counter,
      "VehVActlBrk_No_Cs": brk_cs,
    }))

    # EngVehicleSpThrottle2 (0x202) — second speed source
    msg.append(self.packer.make_can_msg("EngVehicleSpThrottle2", bus0, {
      "Veh_V_ActlEng": speed_kph,
      "VehVActlEng_D_Qf": 3,
    }))

    # Yaw_Data_FD1 (0x91) — yaw rate
    self.yaw_counter = (self.yaw_counter + 1) % 256
    msg.append(self.packer.make_can_msg("Yaw_Data_FD1", bus0, {
      "VehYaw_W_Actl": 0.0,  # no yaw
      "VehRol_W_Actl": 0.0,
      "VehYawWActl_D_Qf": 3,
      "VehRolWActl_D_Qf": 3,
      "VehRollYaw_No_Cnt": self.yaw_counter,
    }))

    # DesiredTorqBrk (0x213) — standstill state
    msg.append(self.packer.make_can_msg("DesiredTorqBrk", bus0, {
      "VehStop_D_Stat": 1 if speed_kph < 0.5 else 0,
      "PrkBrkStatus": 0,
    }))

    # EngVehicleSpThrottle (0x204) — gas pedal
    msg.append(self.packer.make_can_msg("EngVehicleSpThrottle", bus0, {
      "ApedPos_Pc_ActlArb": 10.0 if self.gas_pressed else 0.0,
    }))

    # EngBrakeData (0x165) — brake, cruise state
    cc_stat = 4 if self.cruise_engaged else (3 if self.cruise_available else 0)
    msg.append(self.packer.make_can_msg("EngBrakeData", bus0, {
      "BpedDrvAppl_D_Actl": 2 if self.brake_pressed else 0,
      "CcStat_D_Actl": cc_stat,
      "Veh_V_DsplyCcSet": self.cruise_speed_kph,
      "AccStopMde_D_Rq": 0,
    }))

    # BrakeSnData_4 — brake torque
    msg.append(self.packer.make_can_msg("BrakeSnData_4", bus0, {
      "BrkTot_Tq_Actl": 0,
    }))

    # SteeringPinion_Data — steering angle + quality flag
    msg.append(self.packer.make_can_msg("SteeringPinion_Data", bus0, {
      "StePinComp_An_Est": self.steering_angle,
      "StePinCompAnEst_D_Qf": 3,  # valid
    }))

    # EPAS_INFO — steering torque, fault status, SAPP state
    msg.append(self.packer.make_can_msg("EPAS_INFO", bus0, {
      "SteeringColumnTorque": 0.0,
      "EPAS_Failure": 0,
      "SAPPAngleControlStat1": self.sapp_state,
    }))

    # Cluster_Info1_FD1
    msg.append(self.packer.make_can_msg("Cluster_Info1_FD1", bus0, {
      "DrvSlipCtlMde_D_Rq": 0,
      "AccEnbl_B_RqDrv": 1,
    }))

    # Steering_Data_FD1 — buttons, blinkers
    msg.append(self.packer.make_can_msg("Steering_Data_FD1", bus0, {
      "TurnLghtSwtch_D_Stat": 0,
      "TjaButtnOnOffPress": 0,
      "AccButtnGapTogglePress": 0,
    }))

    # BodyInfo_3_FD1 — doors
    msg.append(self.packer.make_can_msg("BodyInfo_3_FD1", bus0, {
      "DrStatDrv_B_Actl": 0,
      "DrStatPsngr_B_Actl": 0,
      "DrStatRl_B_Actl": 0,
      "DrStatRr_B_Actl": 0,
    }))

    # RCMStatusMessage2_FD1 — seatbelt
    msg.append(self.packer.make_can_msg("RCMStatusMessage2_FD1", bus0, {
      "FirstRowBuckleDriver": 0,  # buckled
    }))

    # PowertrainData_10 — gear
    msg.append(self.packer.make_can_msg("PowertrainData_10", bus0, {
      "TrnRng_D_Rq": 6,  # Drive
    }))

    # INSTRUMENT_PANEL — units
    msg.append(self.packer.make_can_msg("INSTRUMENT_PANEL", bus0, {
      "METRIC_UNITS": 0,  # imperial
    }))

    # === Camera bus (bus 2) ===

    # ACCDATA — from camera
    msg.append(self.packer.make_can_msg("ACCDATA", bus2, {
      "CmbbDeny_B_Actl": 0,
    }))

    # ACCDATA_2 — AEB
    msg.append(self.packer.make_can_msg("ACCDATA_2", bus2, {
      "CmbbBrkDecel_B_Rq": 0,
    }))

    # ACCDATA_3 — TJA/ACC status
    msg.append(self.packer.make_can_msg("ACCDATA_3", bus2, {
      "Tja_D_Stat": 0,
      "FcwVisblWarn_B_Rq": 0,
      "AccStopStat_D_Dsply": 0,
      "AccTrgDist2_D_Dsply": 0,
      "HaDsply_No_Cs": 0,
      "HaDsply_No_Cnt": 0,
    }))

    # IPMA_Data — LKAS status
    msg.append(self.packer.make_can_msg("IPMA_Data", bus2, {
      "LaActvStats_D_Dsply": 0,
      "LaHandsOff_D_Dsply": 0,
    }))

    # Pack and send
    self.pm.send('can', can_list_to_can_capnp(msg))

    self.idx += 1

  def update(self):
    """Read openpilot commands and update simulated vehicle state."""
    self.sm.update(0)

    # Read steering commands from openpilot
    if self.sm.updated['carControl']:
      cc = self.sm['carControl']
      if cc.latActive:
        # Simple sim: apply curvature to steering angle
        self.steering_angle += cc.actuators.curvature * 100.0
        self.steering_angle = max(-500, min(500, self.steering_angle))

  def set_speed(self, speed_kph: float):
    self.speed_kph = speed_kph

  def set_cruise(self, engaged: bool, speed_kph: float = 80.0):
    self.cruise_engaged = engaged
    self.cruise_speed_kph = speed_kph

  def set_sapp_state(self, state: int):
    """Set SAPP handshake state: 0=Closed, 1=Open, 2=Active, 3=Fault"""
    self.sapp_state = state
