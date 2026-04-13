# APA Port — Transit MK5 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port APA (Active Park Assist) steering from phoenixpilot 0.8.0 to modern openpilot for a 2025 Ford Transit MK5. Reference: [design spec](../specs/2026-04-13-apa-port-transit-mk5-design.md).

**Architecture:** Gate on `FORD_TRANSIT_MK5` fingerprint. When factory APA module signals ready via `SAPPAngleControlStat1`, openpilot TXes `ParkAid_Data` (angle) instead of `LateralMotionControl` (curvature). Port phoenixpilot's PSCM handshake state machine and rate limiter verbatim. Safety layer (opendbc/safety/modes/ford.h) whitelists `ParkAid_Data` with angle-based limits.

**Tech Stack:** Python (opendbc), C (opendbc/safety), DBC (ford_lincoln_base_pt.dbc). Safety lives in opendbc_repo now, not panda — panda submodule unchanged.

**Reference source:** `~/ghostdev-apa/selfdrive/car/ford/{carcontroller,carstate,fordcan}.py`

**Repos / branches:**
- `~/openpilot` on `apa` (HEAD `d9675c94a`)
- `~/openpilot/opendbc_repo` on `apa` (HEAD `ab459fa7`)
- panda submodule: **no changes needed** (Ford safety moved to opendbc)

---

## Key facts from exploration

- Modern DBC `ford_lincoln_base_pt.dbc` has all APA signals: `ParkAid_Data` (0x3A8 = 936), `SAPPAngleControlStat1` (in EPAS_INFO), `BrakeSnData_5` (0x76 = 118), `ApaSys_D_Stat`, `EPASExtAngleStatReq`, `ExtSteeringAngleReq2`, `SAPPStatusCoding`, `ApaChime_D_Rq`.
- Modern Ford `interface.py:35` already sets `ret.steerControlType = SteerControlType.angle`.
- Modern carcontroller consumes `actuators.curvature`. Lateral planner populates both `curvature` and `steeringAngleDeg` when `SteerControlType.angle` is set (verify in Task 2).
- `CanBus` abstraction: `.main` (bus 0), `.radar` (bus 1), `.camera` (bus 2). Phoenixpilot TXes `ParkAid_Data` on bus 2 → `CAN.camera`.
- Ford safety is at `opendbc_repo/opendbc/safety/modes/ford.h`, tests at `opendbc_repo/opendbc/safety/tests/test_ford.py`.
- `ExtSteeringAngleReq2` scale: `0.1 deg/bit`, offset `-1000`, range `[-1000, 2276.5]`, bit 22, length 15.

---

## File Structure

**Modify:**
- `opendbc_repo/opendbc/car/ford/values.py` — add `FordFlags.APA`, set on Transit MK5, add APA rate-limit constants
- `opendbc_repo/opendbc/car/ford/carstate.py` — parse `SAPPAngleControlStat1` + APA stock values
- `opendbc_repo/opendbc/car/ford/fordcan.py` — add `create_apa_command`
- `opendbc_repo/opendbc/car/ford/carcontroller.py` — APA branch for Transit, state machine, rate limiter
- `opendbc_repo/opendbc/car/ford/interface.py` — verify angle-control path populates `steeringAngleDeg`
- `opendbc_repo/opendbc/safety/modes/ford.h` — whitelist `ParkAid_Data`, angle checks
- `opendbc_repo/opendbc/safety/tests/test_ford.py` — APA tests
- `opendbc_repo/opendbc/car/ford/tests/test_ford.py` (if exists) — packer/SM tests

**Commit cadence:** one commit per task. Commit in submodule first, then bump submodule ref in outer repo.

---

## Task 1: Add APA flag and rate-limit constants

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/values.py`

- [ ] **Step 1: Add APA flag**

In `class FordFlags(IntFlag)` (around line 51), add:

```python
class FordFlags(IntFlag):
  CANFD = 1
  APA = 2  # Active Park Assist: angle-based steering via ParkAid_Data
```

- [ ] **Step 2: Set APA flag on Transit MK5**

Find the `FORD_TRANSIT_MK5 = FordPlatformConfig(...)` block around line 184. After the `CarSpecs(...)` line, ensure flags are set. Modify:

```python
FORD_TRANSIT_MK5 = FordPlatformConfig(
  [FordCarDocs("Ford Transit 2025", "Co-Pilot360 Assist+")],
  CarSpecs(mass=2068, wheelbase=3.302, steerRatio=16.7),
  flags=FordFlags.APA,
)
```

If `FordPlatformConfig` doesn't accept `flags` as kwarg, check parent class and add assignment in `__post_init__` (mirror how `CANFD` flag is set for CANFDPlatformConfig at values.py:111).

- [ ] **Step 3: Add APA constants to CarControllerParams**

Find `class CarControllerParams` in values.py. Add at the end of the class body (verbatim from phoenixpilot carcontroller.py:13-18):

```python
  # APA (Transit MK5) — values from phoenixpilot 0.8.0
  APA_ANGLE_MAX_BP = [0., 11., 36.]
  APA_ANGLE_MAX_V = [410., 25., 15.]
  APA_ANGLE_DELTA_BP = [0., 5., 15.]
  APA_ANGLE_DELTA_V = [5., .8, .15]     # windup
  APA_ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind
  APA_STEER_STEP = 2                     # 50Hz (every other 100Hz tick)
```

- [ ] **Step 4: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/car/ford/values.py
git commit -m "apa: add APA flag + Transit rate limits"
```

---

## Task 2: Verify lateral planner populates steeringAngleDeg

**Files:**
- Read-only check: `opendbc_repo/opendbc/car/ford/interface.py`, openpilot `selfdrive/controls/` lateral planner

- [ ] **Step 1: Confirm steeringAngleDeg is populated**

Run:
```bash
grep -n "steeringAngleDeg\s*=" ~/openpilot/selfdrive/controls/lib/latcontrol*.py
```

Expected: a lat controller (e.g. `latcontrol_angle.py`) assigns `actuator.steeringAngleDeg` when `CP.steerControlType == angle`. If not found, angle is populated elsewhere (e.g. `controlsd.py`) — trace and confirm. Document the source in a comment in the upcoming carcontroller change (Task 5).

- [ ] **Step 2: No commit** — this is a verification task.

---

## Task 3: Carstate — parse APA signals

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/carstate.py`

- [ ] **Step 1: Add APA fields to CarState class init**

In `class CarState(CarStateBase):`, add to `__init__` (or add `__init__` if none exists):

```python
  def __init__(self, CP):
    super().__init__(CP)
    self.apa_handshake = 0   # SAPPAngleControlStat1: 0=Closed 1=Open 2=Active 3=Fault
    self.apa_sys_stat = 0    # ApaSys_D_Stat: 0=Null 1=Off 2=On 3=Overspeed 4=Cancelled 5=NotAccessible 6=Finished 7=Faulty
    self.apa_angle_stat = 0  # EPASExtAngleStatReq echo
```

- [ ] **Step 2: Parse EPAS_INFO + ParkAid_Data in update()**

In `CarState.update()`, after existing EPAS_INFO reads (around line 50), add APA-specific reads gated on `FordFlags.APA`:

```python
    if self.CP.flags & FordFlags.APA:
      self.apa_handshake = cp_cam.vl["EPAS_INFO"]["SAPPAngleControlStat1"]
      self.apa_sys_stat = cp.vl["ParkAid_Data"]["ApaSys_D_Stat"]
      self.apa_angle_stat = cp.vl["ParkAid_Data"]["EPASExtAngleStatReq"]
```

Import `FordFlags` at the top if not already imported.

- [ ] **Step 3: Add signals to parsers**

In `get_can_parser(CP)` (main bus parser), add inside the `messages` list (gated by APA flag or always-present since Transit is APA):

```python
    if CP.flags & FordFlags.APA:
      messages += [
        ("ParkAid_Data", 50),  # 20ms = 50Hz per DBC GenMsgCycleTime
      ]
```

In `get_cam_can_parser(CP)` (camera bus parser), ensure `EPAS_INFO` is parsed with `SAPPAngleControlStat1`. Check existing signals — if EPAS_INFO is already parsed, verify the signal is requested. If not, add:

```python
    if CP.flags & FordFlags.APA:
      messages += [("EPAS_INFO", 100)]  # verify freq from DBC
```

Verify EPAS_INFO frequency:
```bash
grep "EPAS_INFO" ~/openpilot/opendbc_repo/opendbc/dbc/ford_lincoln_base_pt.dbc | grep -i cycle
```

Use actual cycle time. Modern parser API uses `(name, frequency_hz)` tuples — match the style used by surrounding entries in this file.

- [ ] **Step 4: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/car/ford/carstate.py
git commit -m "apa: parse SAPPAngleControlStat1 and ParkAid_Data echo"
```

---

## Task 4: fordcan — create_apa_command packer

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/fordcan.py`

- [ ] **Step 1: Add `create_apa_command` function**

Append to `fordcan.py` (signal list from phoenixpilot fordcan.py:4-24, verified against modern DBC ParkAid_Data at ford_lincoln_base_pt.dbc:2808):

```python
def create_apa_command(packer, CAN: CanBus, apply_angle: float, angle_req: bool,
                      sapp_config: int, sapp_action: int, sapp_chime: int = 0):
  """
  Creates CAN message for Ford Active Park Assist angle command.

  ParkAid_Data (0x3A8) is TX'd on the camera bus (bus 2) at 50Hz (GenMsgCycleTime=20ms).
  PSCM accepts this angle when SAPPAngleControlStat1 handshake completes.

  Args:
    apply_angle: desired wheel angle in degrees (rate-limited upstream)
    angle_req: EPASExtAngleStatReq (1=Request, 0=NoRequest)
    sapp_config: SAPPStatusCoding — handshake coding (see carcontroller state machine)
    sapp_action: ApaSys_D_Stat — 0..7 (see DBC VAL_TABLE ApaSys_D_Stat)
    sapp_chime: ApaChime_D_Rq — 0..7

  Frequency: 50Hz. Bus: camera (2).
  """
  values = {
    "ApaSys_D_Stat": sapp_action,
    "EPASExtAngleStatReq": 1 if angle_req else 0,
    "ExtSteeringAngleReq2": apply_angle,
    "SAPPStatusCoding": sapp_config,
    "ApaChime_D_Rq": sapp_chime,
  }
  return packer.make_can_msg("ParkAid_Data", CAN.camera, values)
```

- [ ] **Step 2: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/car/ford/fordcan.py
git commit -m "apa: add create_apa_command packer"
```

---

## Task 5: Carcontroller — APA state machine + rate limiter

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/carcontroller.py`

This is the biggest change. Port phoenixpilot's APA SM into a method; call it instead of curvature path when `FordFlags.APA` is set.

- [ ] **Step 1: Add APA state to __init__**

In `CarController.__init__`, append:

```python
    # APA state (Transit MK5) — mirrors phoenixpilot 0.8.0 handshake
    self.apa_last_angle = 0.0
    self.apa_counter = 0
    self.apa_eightysix = 0
    self.apa_sapp_config = 0
    self.apa_sapp_config_last = 0
    self.apa_angle_req = 0
    self.apa_angle_req_last = 0
    self.apa_sapp_action = 0
    self.apa_sys_stat = 0  # ApaSys_D_Stat we TX — 6=Finished means "openpilot not requesting"
```

- [ ] **Step 2: Add apa_update method**

Add to `CarController` class (port of phoenixpilot carcontroller.py:119-172):

```python
  def apa_update(self, CC, CS, can_sends):
    """
    Active Park Assist control for Transit MK5.

    Sends ParkAid_Data at 50Hz with a PSCM handshake state machine:
      CS.apa_handshake: 0=Closed, 1=Open, 2=Active, 3=Fault
      SAPPStatusCoding cycle: 0 -> 70 -> 86 -> 224 -> 16 (steady state)

    Rate-limits desired angle per phoenixpilot windup/unwind tables.
    """
    from opendbc.car.common.numpy_fast import clip, interp

    enabled = CC.latActive
    apply_angle = CC.actuators.steeringAngleDeg

    # Only run at STEER_STEP cadence (50Hz if STEER_STEP=2 and controls tick is 100Hz)
    if (self.frame % CarControllerParams.APA_STEER_STEP) != 0:
      return

    if not enabled:
      self.apa_counter = 0
      self.apa_eightysix = 0
      self.apa_angle_req = 0
      self.apa_sapp_action = 0
    else:
      self.apa_counter += 1

      # init handshake: send config 70 until PSCM responds
      if CS.apa_handshake == 0 and self.apa_sapp_config_last not in (16, 86, 224):
        self.apa_sapp_config = 70

      # PSCM acks (handshake=1); after 8 frames at ack, advance to config 86
      if CS.apa_handshake == 1 and self.apa_counter > 8:
        self.apa_sapp_config = 86
        self.apa_eightysix += 1

      # 5 more frames at 86, start requesting angle
      if CS.apa_handshake == 1 and self.apa_counter > 13 and self.apa_sapp_config_last == 86:
        self.apa_angle_req = 1

      # 20 frames at 86: reset counters, keep angle request
      if self.apa_sapp_config_last == 86 and self.apa_eightysix == 20:
        self.apa_counter = 0
        self.apa_eightysix = 0
        self.apa_angle_req = 1

      # PSCM enters Active (handshake=2): advance to config 224
      if CS.apa_handshake == 2 and self.apa_sapp_config_last != 16:
        self.apa_sapp_config = 224
        self.apa_angle_req = 1
        self.apa_sapp_action += 1

      # after 3 frames at 224, move to steady config 16
      if CS.apa_handshake == 2 and self.apa_sapp_action >= 3 and self.apa_sapp_config_last == 224:
        self.apa_sapp_config = 16
        self.apa_angle_req = 1

      # PSCM fault: reset and retry
      if CS.apa_handshake == 3:
        self.apa_sapp_config = 0
        self.apa_counter = 0
        self.apa_angle_req = 0

    self.apa_sapp_config_last = self.apa_sapp_config
    self.apa_angle_req_last = self.apa_angle_req

    # angle clip by speed
    angle_lim = interp(CS.out.vEgo, CarControllerParams.APA_ANGLE_MAX_BP,
                       CarControllerParams.APA_ANGLE_MAX_V)
    apply_angle = clip(apply_angle, -angle_lim, angle_lim)

    # rate limit by speed (windup vs unwind)
    if enabled:
      if self.apa_last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.apa_last_angle):
        rate_lim = interp(CS.out.vEgo, CarControllerParams.APA_ANGLE_DELTA_BP,
                          CarControllerParams.APA_ANGLE_DELTA_V)  # windup
      else:
        rate_lim = interp(CS.out.vEgo, CarControllerParams.APA_ANGLE_DELTA_BP,
                          CarControllerParams.APA_ANGLE_DELTA_VU)  # unwind
      apply_angle = clip(apply_angle, self.apa_last_angle - rate_lim,
                                       self.apa_last_angle + rate_lim)
    else:
      apply_angle = CS.out.steeringAngleDeg  # follow measured when disengaged

    self.apa_last_angle = apply_angle

    # ApaSys_D_Stat: phoenix sends 0 ("Null") as lkas_action; keep same for parity
    sapp_action = 0
    can_sends.append(fordcan.create_apa_command(
      self.packer, self.CAN, apply_angle, self.apa_angle_req_last,
      self.apa_sapp_config_last, sapp_action,
    ))
```

NOTE: if `opendbc.car.common.numpy_fast` does not exist in modern opendbc, replace `clip` with `float(np.clip(...))` and `interp` with `float(np.interp(...))` — `np` is already imported.

- [ ] **Step 3: Branch in update() — APA vs curvature**

Around carcontroller.py:100 (`### lateral control ###` block), replace the block with:

```python
    ### lateral control ###
    if self.CP.flags & FordFlags.APA:
      self.apa_update(CC, CS, can_sends)
    else:
      # existing curvature path — keep unchanged for other Ford platforms
      if (self.frame % CarControllerParams.STEER_STEP) == 0:
        # ... existing code from lines 103-128 ...
```

Keep the entire existing curvature block inside the `else` branch. Do not delete anything.

Also, skip `create_lka_msg` for APA platforms — the `Lane_Assist_Data1` message is not used; the LKA UI msg sending block can stay since it populates HUD state, but `create_lka_msg` at line 132 should be gated:

```python
    # send lka msg at 33Hz — non-APA platforms only
    if not (self.CP.flags & FordFlags.APA) and (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))
```

- [ ] **Step 4: Update new_actuators return**

At end of `update()` around line 197:

```python
    new_actuators = actuators.as_builder()
    if self.CP.flags & FordFlags.APA:
      new_actuators.steeringAngleDeg = self.apa_last_angle
    else:
      new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
```

- [ ] **Step 5: Import FordFlags**

Verify top of file has `from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR`. Already present.

- [ ] **Step 6: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/car/ford/carcontroller.py
git commit -m "apa: add APA state machine + angle rate limiter for Transit"
```

---

## Task 6: Safety — whitelist ParkAid_Data TX + angle checks

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/ford.h`

- [ ] **Step 1: Add ParkAid_Data address**

After line 19 (other `FORD_*` address defines), add:

```c
#define FORD_ParkAid_Data          0x3A8U   // TX by OP on APA platforms, angle-based park assist
```

(0x3A8 = 936 decimal, confirmed from DBC.)

- [ ] **Step 2: Add APA steering limits**

After `FORD_STEERING_LIMITS` (line 111), add angle-based limits. ExtSteeringAngleReq2 has scale 0.1 deg/bit offset -1000:

```c
// APA (Ford Transit MK5): angle-based control via ExtSteeringAngleReq2
// Scale: 0.1 deg/bit, offset -1000 (so raw = (angle_deg + 1000) * 10)
// Limits from phoenixpilot 0.8.0: max |angle| 410deg at 0mph, 15deg at 36mph
static const AngleSteeringLimits FORD_APA_STEERING_LIMITS = {
  .max_angle = 4100,          // 410 degrees (raw) at 0 mph
  .angle_deg_to_can = 10,     // 0.1 deg/bit
  .max_angle_error = 50,      // 5 deg tolerance
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {5., 0.8, 0.15},           // phoenixpilot APA_ANGLE_DELTA_V (windup)
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {5., 3.5, 0.4},            // phoenixpilot APA_ANGLE_DELTA_VU (unwind)
  },
  .angle_error_min_speed = 100.0,  // effectively disable error check (APA is parking speed)
  .angle_is_curvature = false,
  .enforce_angle_error = false,    // phoenixpilot did not enforce; PSCM has its own checks
  .inactive_angle_is_zero = false, // APA can disengage with non-zero commanded angle
};
```

- [ ] **Step 3: Add TX hook for ParkAid_Data**

After the `FORD_LateralMotionControl2` block in `ford_tx_hook` (around line 280):

```c
  // Safety check for ParkAid_Data (APA angle command)
  if (msg->addr == FORD_ParkAid_Data) {
    // Signal: EPASExtAngleStatReq (bit 23, 1 bit)
    bool steer_control_enabled = ((msg->data[2] >> 7) & 1U) != 0U;
    // Signal: ExtSteeringAngleReq2 (bit 22 MSB, 15 bits, big-endian, scale 0.1, offset -1000)
    // In CAN payload: bits 22..8 across bytes 2-3
    unsigned int raw_angle = (((unsigned int)(msg->data[2] & 0x7FU)) << 8) | msg->data[3];
    // Convert to signed angle in 0.1deg units relative to zero
    int desired_angle = (int)raw_angle - 10000;  // 10000 raw = 0 deg per offset -1000, scale 0.1

    bool violation = steer_angle_cmd_checks(desired_angle, steer_control_enabled,
                                            FORD_APA_STEERING_LIMITS);
    if (violation) {
      tx = false;
    }
  }
```

**IMPORTANT:** The bit-extraction above must match the DBC byte ordering. Ford DBC uses big-endian (Motorola) signals. `ExtSteeringAngleReq2 : 22|15@0+` means start bit 22, length 15, big-endian, unsigned. Double-check extraction by packing a known angle in Python and verifying the C extraction reproduces the same raw value.

Verification snippet (run after Task 4):
```python
from opendbc.can import CANPacker
p = CANPacker("ford_lincoln_base_pt")
msg = p.make_can_msg("ParkAid_Data", 2, {"ExtSteeringAngleReq2": 0.0, "EPASExtAngleStatReq": 1})
print([hex(b) for b in msg[2]])  # observe bytes; compute raw_angle from bytes using the C formula
```

If the C extraction doesn't match, fix the bit shifts.

- [ ] **Step 4: Add ParkAid_Data to TX whitelist**

In `ford_init`, add a new APA TX list or extend `FORD_LONG_TX_MSGS`. Since APA is CAN (not CANFD) and replaces LateralMotionControl, best to define a new list:

```c
  static const CanMsg FORD_APA_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_ParkAid_Data, 2, 8, .check_relay = true},  // TX on camera bus (bus 2)
  };
```

Add a param flag for APA:

```c
  const uint16_t FORD_PARAM_APA = 4;
  const bool ford_apa = GET_FLAG(param, FORD_PARAM_APA);
```

And in the selection block:

```c
  if (ford_canfd) {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
  } else if (ford_apa) {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_APA_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }
```

- [ ] **Step 5: Wire param flag in Ford values.py**

In `opendbc/car/ford/values.py`, find `FordSafetyFlags` enum. Add:

```python
class FordSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  CANFD = 2
  APA = 4
```

In `interface.py` `_get_params`, set the flag when `FordFlags.APA` is in CP.flags:

```python
    if ret.flags & FordFlags.APA:
      ret.safetyConfigs[0].safetyParam |= FordSafetyFlags.APA.value
```

Find appropriate location in interface.py by grepping for existing `safetyParam` assignments.

- [ ] **Step 6: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/safety/modes/ford.h opendbc/car/ford/values.py opendbc/car/ford/interface.py
git commit -m "apa: safety — whitelist ParkAid_Data with angle limits"
```

---

## Task 7: Safety tests — ParkAid_Data TX

**Files:**
- Modify: `opendbc_repo/opendbc/safety/tests/test_ford.py`

- [ ] **Step 1: Read existing ford safety test structure**

```bash
grep -n "class\|def test_\|_make_msg\|LateralMotionControl\|lat_ctl" ~/openpilot/opendbc_repo/opendbc/safety/tests/test_ford.py | head -40
```

Identify the test class pattern used for LateralMotionControl. Replicate for APA.

- [ ] **Step 2: Add APA test class**

Add a new test class mirroring `TestFordStockSafety` (or similar), parameterized with `FORD_PARAM_APA`:

```python
class TestFordAPASafety(TestFordSafetyBase):
  TX_MSGS = [
    # common
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2],
    [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0], [MSG_IPMA_Data, 0],
    [MSG_ACCDATA, 0],
    [MSG_ParkAid_Data, 2],
  ]

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ParkAid_Data, MSG_IPMA_Data, MSG_ACCDATA_3, MSG_Lane_Assist_Data1]}

  def setUp(self):
    self.packer = CANPackerPanda("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.APA)
    self.safety.init_tests()

  def _apa_msg(self, angle: float, req: bool):
    values = {
      "ExtSteeringAngleReq2": angle,
      "EPASExtAngleStatReq": 1 if req else 0,
      "SAPPStatusCoding": 16,
      "ApaSys_D_Stat": 0,
      "ApaChime_D_Rq": 0,
    }
    return self.packer.make_can_msg_panda("ParkAid_Data", 2, values)

  def test_apa_tx_allowed_when_controls_allowed(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._apa_msg(0.0, True)))

  def test_apa_tx_blocked_when_controls_not_allowed(self):
    self.safety.set_controls_allowed(False)
    # must send angle=0 + req=0 when disengaged per inactive_angle_is_zero=false handling
    self.assertTrue(self._tx(self._apa_msg(0.0, False)))   # inactive is OK
    self.assertFalse(self._tx(self._apa_msg(50.0, True)))  # active with nonzero blocked

  def test_apa_angle_limit(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._apa_msg(400.0, True)))
    self.assertFalse(self._tx(self._apa_msg(500.0, True)))  # > 410 max

  def test_apa_rate_limit(self):
    self.safety.set_controls_allowed(True)
    # step 10 deg/frame at standstill is within rate limit (5 deg/frame windup per APA_ANGLE_DELTA_V[0])
    self.assertTrue(self._tx(self._apa_msg(0.0, True)))
    self.assertTrue(self._tx(self._apa_msg(5.0, True)))
    # 20 deg jump should violate rate limit
    self.assertFalse(self._tx(self._apa_msg(25.0, True)))
```

Add `MSG_ParkAid_Data = 0x3A8` constant near other `MSG_*` constants at top of file.

**Note:** exact helper names (`_tx`, `_make_msg`, test base class, etc.) may differ. Read existing tests first and adapt signatures accordingly. The above is the test intent; wire to actual API.

- [ ] **Step 3: Run tests**

```bash
cd ~/openpilot/opendbc_repo
pytest opendbc/safety/tests/test_ford.py -v -k APA
```

Expected: all APA tests pass. If rate-limit test fails, compare units — safety uses `angle_deg_to_can = 10` (0.1 deg), so 5 deg/frame = 50 raw/frame. Adjust test values if needed.

- [ ] **Step 4: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/safety/tests/test_ford.py
git commit -m "apa: safety tests for ParkAid_Data angle limits"
```

---

## Task 8: Packer round-trip test

**Files:**
- Create or modify: `opendbc_repo/opendbc/car/ford/tests/test_ford.py` (if exists, else create)

- [ ] **Step 1: Locate Ford car tests**

```bash
ls ~/openpilot/opendbc_repo/opendbc/car/ford/tests/
```

- [ ] **Step 2: Add packer test**

In appropriate test file (create if none), add:

```python
def test_create_apa_command_roundtrip():
  from opendbc.can import CANPacker, CANParser
  from opendbc.car.ford.fordcan import CanBus, create_apa_command

  packer = CANPacker("ford_lincoln_base_pt")
  CAN = CanBus()  # main=0, camera=2

  addr, _, data, bus = create_apa_command(packer, CAN, apply_angle=42.5, angle_req=True,
                                           sapp_config=86, sapp_action=2, sapp_chime=0)

  assert bus == 2
  assert addr == 0x3A8

  parser = CANParser("ford_lincoln_base_pt", [("ParkAid_Data", 0)], 2)
  parser.update([(addr, 0, data, bus)])
  vals = parser.vl["ParkAid_Data"]
  assert abs(vals["ExtSteeringAngleReq2"] - 42.5) < 0.1
  assert vals["EPASExtAngleStatReq"] == 1
  assert vals["SAPPStatusCoding"] == 86
  assert vals["ApaSys_D_Stat"] == 2
```

Modern CANParser/CANPacker API may differ — mirror patterns from existing tests in the same directory.

- [ ] **Step 3: Run test**

```bash
cd ~/openpilot/opendbc_repo
pytest opendbc/car/ford/tests/ -v -k apa
```

Expected: pass.

- [ ] **Step 4: Commit**

```bash
cd ~/openpilot/opendbc_repo
git add opendbc/car/ford/tests/
git commit -m "apa: packer round-trip test for create_apa_command"
```

---

## Task 9: Bump submodule in openpilot

**Files:**
- Modify: `~/openpilot/opendbc_repo` submodule ref

- [ ] **Step 1: Push opendbc apa branch**

```bash
cd ~/openpilot/opendbc_repo
git push origin apa
```

- [ ] **Step 2: Bump submodule in outer repo**

```bash
cd ~/openpilot
git add opendbc_repo
git commit -m "apa: bump opendbc submodule with APA port"
```

- [ ] **Step 3: Push apa branch**

```bash
cd ~/openpilot
git push origin apa
```

---

## Task 10: Integration dry-run — CI-style full sim check

**Files:**
- No changes — validation only

- [ ] **Step 1: Ford unit tests**

```bash
cd ~/openpilot/opendbc_repo
pytest opendbc/car/ford/ -v
pytest opendbc/safety/tests/test_ford.py -v
```

Expected: all pass. Non-APA Ford tests must not regress.

- [ ] **Step 2: openpilot car interface test**

```bash
cd ~/openpilot
python -c "
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
ret = CarInterface.get_non_essential_params(CAR.FORD_TRANSIT_MK5)
print('steerControlType:', ret.steerControlType)
print('flags:', ret.flags)
print('safetyConfigs:', ret.safetyConfigs)
"
```

Expected: `steerControlType: angle`, `flags` includes APA bit, safety param includes APA bit.

- [ ] **Step 3: Lint/type check**

```bash
cd ~/openpilot/opendbc_repo
ruff check opendbc/car/ford/ opendbc/safety/tests/test_ford.py
```

Fix any findings.

- [ ] **Step 4: No commit** — validation only. If anything fails, fix in relevant task and re-run.

---

## Task 11: On-vehicle bring-up checklist

**No code changes.** This is the validation protocol; the driver executes it. Document in a commit message or notes for driver.

- [ ] **Step 1: Power-on sanity**
  - Boot comma device on Transit, verify car fingerprinted as `FORD_TRANSIT_MK5`
  - Cabana/ui: confirm `SAPPAngleControlStat1` decoded, `ParkAid_Data` RX'd from PSCM

- [ ] **Step 2: Static handshake test**
  - In Park, engine on, factory APA dash button pressed
  - Verify CS.apa_handshake transitions 0 → 1 → 2
  - Verify openpilot TX `ParkAid_Data` frames on bus 2 (check with scope/panda logs)

- [ ] **Step 3: Low-speed angle tracking**
  - Put in R or D, crawl speed (<3 mph), openpilot engaged
  - Command small angle from lateral planner — wheel tracks within ~2 degrees
  - Abort on any unexpected behavior

- [ ] **Step 4: Driver override**
  - Grab wheel firmly during engaged APA — PSCM should fault, carstate shows apa_handshake=3
  - Openpilot SM resets; disengage cleanly

- [ ] **Step 5: Clean disengage**
  - Press factory button or shift to P — `ParkAid_Data` goes to inactive frame
  - Verify panda does not fault

If any step fails, diagnose and iterate.

---

## Self-review notes (do not skip)

Before execution, auditor/implementer should verify:

1. **DBC byte ordering in safety C code (Task 6 Step 3):** phoenixpilot is Python (packer handles endianness); C must manually extract `ExtSteeringAngleReq2` (bit 22, len 15, big-endian). Step 3 includes a verification snippet — **use it**, do not skip.
2. **Modern opendbc `numpy_fast` availability (Task 5):** may not exist. Fall back to `np.clip`/`np.interp`.
3. **Parser message API (Task 3):** modern opendbc may use `messages` list of `(name, freq)` tuples, not `signals`/`checks`. Mirror existing Ford carstate patterns.
4. **`CarController.__init__` signature (Task 5):** modern signature is `(self, dbc_names, CP)`, phoenixpilot was `(self, dbc_name, CP, VM)`. No VM available — if `VM.get_steer_from_curvature` is needed, it is not needed here since we consume `actuators.steeringAngleDeg` directly.
5. **`CarState.__init__` signature (Task 3):** may be `(self, CP)` — check and call `super().__init__(CP)`.
6. **PSCM-happy messages (0x202, 0x415, BrakeSnData_5) from phoenixpilot:** NOT ported. Rationale: in the relay-based modern panda architecture on an unmodified Transit, those messages originate from stock ECUs and pass through the relay. Phoenix spoofed them because its bench had a disconnected ABS/PCM. **Bring-up validates this assumption.** If PSCM rejects steering due to missing counters, revisit and port those messages to carcontroller (gated on APA flag, TX'd on camera bus).
7. **Handshake magic numbers (70, 86, 224, 16)** are reverse-engineered values from phoenix — no documentation. Port verbatim, do not reinterpret.

---

## Execution choice

Plan complete and saved to `docs/superpowers/plans/2026-04-13-apa-port-transit-mk5.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks. Tasks here are file-scoped with concrete code in each step, ideal for subagent execution. Tasks 6 and 7 (C safety + tests) are the highest-risk — manual review recommended there.

**2. Inline Execution** — Execute tasks in this session. Faster for tight iteration but burns context.

Which approach?
