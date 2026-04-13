# APA Port — 2025 Ford Transit MK5

Status: design approved, pending user review of written spec
Target branches: `ghostdev137/openpilot@apa`, `ghostdev137/opendbc@apa`, `ghostdev137/panda@apa` (to create)
Source port: `~/ghostdev-apa/` (openpilot 0.8.0, phoenixpilot ford-devel-apa-080)

## Goal

Land a drive-ready Active Park Assist implementation for a 2025 Ford Transit MK5 on the `apa` fork. Openpilot commands steering angle via `ParkAid_Data` when the factory APA module requests control (`SAPPAngleControlStat1` ready). The Transit has factory ACC (used as-is) and no LCA, so APA is the sole lateral control path. Logic ports 1:1 from phoenixpilot 0.8.0 since PSCM behavior is identical.

## Non-goals

- Openpilot-initiated parking (no UI button; factory APA module triggers)
- Longitudinal control (factory ACC handles it)
- Coexistence with curvature-based lateral control (Transit has no LCA)
- Highway lateral control on this fork

## Architecture

```
EPAS_INFO CAN RX
  └─ carstate parses SAPPAngleControlStat1 → CS.out (APA ready/active/fault)

controlsd (angle-control platform flag)
  └─ populates actuators.steeringAngleDeg

carcontroller APA state machine
  ├─ if CS.apa_ready and CC.latActive:
  │    apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, ...)
  │    emit ParkAid_Data via create_apa_command
  └─ else: send disengaged/standby ParkAid_Data frame

panda safety_ford.h
  └─ validates ParkAid_Data angle/rate, honors driver override → TX
```

## Components

### 1. opendbc (`ghostdev137/opendbc@apa`, base `ab459fa7`)

**`opendbc/car/ford/values.py`**
- Add `CarControllerParams.STEER_ANGLE_LIMITS` / rate limits using phoenixpilot values (verbatim)
- Set Transit MK5 `CarSpecs`/platform flag to mark as angle-controlled so controlsd populates `steeringAngleDeg`

**`opendbc/car/ford/carstate.py`**
- Parse `SAPPAngleControlStat1` from `EPAS_INFO` (address already in modern DBC)
- Expose APA state (ready/active/fault) via `CS.out` fields or a carstate attribute used by carcontroller

**`opendbc/car/ford/fordcan.py`**
- Add `create_apa_command(packer, apply_angle, enabled)` packing `ParkAid_Data` fields per phoenixpilot source (`ApaSys_D_Stat`, `LatCtlRng_L_Max`, `ApaPrkNudg_B_Rq`, lateral motion control fields)
- Preserve existing `create_lat_ctl*` helpers (unused on Transit but kept for other Ford platforms on this fork)

**`opendbc/car/ford/carcontroller.py`**
- Replace curvature-based TX path with APA path for Transit MK5
- Port phoenixpilot APA state machine (OFF → STANDBY → ACTIVE → FAULT)
- Use `apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, ANGLE_LIMITS)` (pattern from Nissan/PSA)
- `new_actuators.steeringAngleDeg = self.apply_angle_last` so controlsd sees what was commanded
- Gate on Transit MK5 platform; other Ford platforms keep existing behavior

**`opendbc/car/ford/tests/`**
- Packer round-trip test: `create_apa_command` → parser returns same fields
- State machine unit test: OFF → STANDBY → ACTIVE transitions given mock CS inputs
- Existing safety tests still pass

### 2. panda (`ghostdev137/panda@apa`, fork off commaai `18f37937`)

**`board/safety/safety_ford.h`**
- Add `FORD_PARKAID_DATA` addr to `FORD_TX_MSGS`
- Add `steer_angle_cmd_checks` invocation for `ParkAid_Data` using `SteeringLimits` struct (angle max, rate up/down, max-angle-at-speed curves) — values from phoenixpilot
- Driver-override: check `SAPPAngleControlStat1` driver-torque bit; on override, freeze/fault APA TX

**`tests/safety/test_ford.py`**
- Allow: `ParkAid_Data` when controls_allowed
- Block: `ParkAid_Data` when controls_allowed=0, angle > limit, rate > limit, driver override
- Round-trip: existing Ford tests pass

### 3. openpilot (`ghostdev137/openpilot@apa`, base `b263b7cf4`)

- Bump `opendbc_repo` submodule to new apa tip
- Change `panda` submodule URL to `ghostdev137/panda` and bump to new apa tip
- Verify no stale APA references in `selfdrive/` wiring (already stripped per recent commits)
- Update `.gitmodules` if panda URL changes

## Data flow detail

`CS.out.steeringAngleDeg`: measured wheel angle from existing `SteeringPinion_Data` (unchanged).

`actuators.steeringAngleDeg`: desired angle, populated by controlsd's lateral planner when platform is flagged angle-control. Same signal Nissan/Tesla/PSA consume.

`CS.apa_ready` (new): derived from `SAPPAngleControlStat1`. Carcontroller uses this to gate TX.

`apply_angle_last`: last commanded angle, used by `apply_std_steer_angle_limits` for rate limiting and echoed back via `new_actuators`.

## Testing

**Unit / CI**
- opendbc Ford tests (existing + new APA packer/SM tests)
- panda Ford safety tests (existing + new `ParkAid_Data` tests)

**On-vehicle bring-up checklist** (cannot automate)
1. Power on, verify `SAPPAngleControlStat1` decoded in carstate (cabana/live)
2. Transit in park, engage APA from factory dash button — confirm CS.apa_ready goes true
3. Verify openpilot TX `ParkAid_Data` frames appear on bus only when APA ready + engaged
4. Begin parking maneuver at crawl speed — wheel tracks commanded angle
5. Driver override: grab wheel — panda blocks TX, APA faults cleanly
6. Disengage: factory button or shift out — `ParkAid_Data` goes to standby frame

## Open questions resolved

- Activation model: factory-initiated passthrough (A)
- Panda safety: full unblock + limits + tests (B)
- Lateral coexistence: n/a (no LCA on Transit)
- Longitudinal: factory ACC, no OP involvement
- Angle source: `actuators.steeringAngleDeg` (same as Tesla/Nissan/PSA)
- Angle/rate limits: phoenixpilot values verbatim
- Panda branch: create `ghostdev137/panda@apa` off `18f37937`

## Risks

- **Platform-flag branching**: if Ford carcontroller is flagged angle-control globally, Escape/Bronco/etc. on the fork break. Mitigation: gate on Transit MK5 platform only, other Fords stay curvature-based.
- **SAPPAngleControlStat1 signal naming**: if modern DBC renamed the signal from 0.8.0, carstate parse needs the current name. Verify during implementation.
- **ParkAid_Data field set**: modern DBC may have added/renamed fields vs. 0.8.0. Packer must match current DBC, not phoenixpilot DBC verbatim.
