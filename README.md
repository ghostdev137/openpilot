# ghostpilot

A Ford-focused openpilot fork with full CAN passthrough, no telemetry, and experimental steering modes for development and research.

Based on [commaai/openpilot](https://github.com/commaai/openpilot). Not affiliated with comma.ai.

## What's Different

### Telemetry Disabled
All phone-home behavior removed:
- Sentry crash reporting neutered (logs locally only)
- athenad (comma.ai websocket), uploader, statsd, tombstoned, logmessaged, feedbackd all disabled
- Registration failure no longer crashes — runs as unregistered device
- Setup script sentry events disabled

### Panda Safety: Full Passthrough
Ford safety mode is fully open:
- `ford_tx_hook` always returns true — no accel, steering, or button checks
- `ford_rx_hook` always sets `controls_allowed = true`
- `ford_init` returns null config — no TX whitelist, no RX checks
- `safety_tx_hook` treats `SAFETY_FORD` same as `SAFETY_ALLOUTPUT`
- No relay malfunction checks
- **Any CAN message, any bus, any value**

### Ford Transit Support
- Platform: `FORD_TRANSIT_MK5` (2025 Ford Transit)
- CAN bus (Q3 harness), not CAN FD
- Specs: 2500kg, 3.30m wheelbase, 17.5 steer ratio (estimates, refine on-vehicle)
- Fingerprints: empty — capture from vehicle with `fw_query`

### Steering Modes
Three lateral control modes, selectable via a single constant:

| Mode | Channel | How it works |
|------|---------|-------------|
| **STOCK** (default) | TJA/LCA curvature | Standard openpilot lane centering via `LateralMotionControl` |
| **APA** | ParkAid_Data (0x3A8) | SAPP handshake with PSCM for direct steering angle control |
| **LKA** | Lane_Assist_Data1 (0x3CA) | Incremental angle corrections through the LKA channel |

#### Switching Modes
Edit `opendbc_repo/opendbc/car/ford/values.py`, in `CarControllerParams.__init__`:
```python
self.STEERING_MODE = SteeringMode.STOCK  # default
self.STEERING_MODE = SteeringMode.APA    # SAPP angle control
self.STEERING_MODE = SteeringMode.LKA    # LKA angle control
```

#### APA Mode Details
Uses the SAPP (Semi-Automatic Parallel Parking) protocol to get direct steering angle control from the PSCM:

1. Sends `SAPPStatusCoding = 70` to initiate handshake
2. Waits for PSCM to respond with `SAPPAngleControlStat1 = 1` (Open)
3. Progresses through config values 86 → 224 → 16
4. Once `SAPPAngleControlStat1 = 2` (Active), PSCM accepts angle commands via `ExtSteeringAngleReq2`
5. On fault (state 3), resets and retries automatically

State machine: `IDLE → INIT → WAIT_OPEN → ANGLE_REQ → WAIT_ACTIVE → PARALLEL → COMPLETE`

#### LKA Mode Details
Sends incremental steering angle corrections through `Lane_Assist_Data1`:
- Computes relative angle: `desired - current`
- Clips to +/-5.8 degrees, converts to milliradians
- Sets direction (2=left, 4=right) and ramp type
- Sends inactive `LateralMotionControl` to keep CAN bus happy

## Setup

```bash
# Clone
git clone https://github.com/ghostdev137/openpilot.git ~/openpilot
cd ~/openpilot
git checkout ghostpilot

# Setup
OPENPILOT_ROOT=~/openpilot bash tools/op.sh setup

# Build (including panda firmware)
source .venv/bin/activate
scons -j$(nproc)
cd panda && scons -j$(nproc) && cd ..

# Test
python -m pytest selfdrive/car/tests/test_car_interfaces.py -x -q
```

## Staying in Sync with Upstream

```bash
git fetch upstream
git merge upstream/master
```

## Related Repos

| Repo | Purpose |
|------|---------|
| [ghostdev137/openpilot](https://github.com/ghostdev137/openpilot) | This fork (branch: ghostpilot) |
| [ghostdev137/opendbc](https://github.com/ghostdev137/opendbc) | Fork of commaai/opendbc (branch: ghostpilot) |
| [ghostdev137/panda](https://github.com/ghostdev137/panda) | Fork of commaai/panda (branch: ghostpilot) |
| [ghostdev137/apa](https://github.com/ghostdev137/apa) | Reference: roxasthenobody98 APA fork (ford-devel-apa-080) |
| [ghostdev137/lane-assist-driving](https://github.com/ghostdev137/lane-assist-driving) | Reference: mims002 LKA fork |

## On-Vehicle Validation Needed

Before real use, confirm on your vehicle:
- [ ] LKA direction values (2=left, 4=right may be inverted)
- [ ] APA `ApaSys_D_Stat` value during handshake
- [ ] Transit steer ratio and wheelbase
- [ ] Capture ECU fingerprints (EPS, ABS, radar, camera) via `fw_query`

## Key Files

| File | What |
|------|------|
| `opendbc_repo/opendbc/car/ford/carcontroller.py` | Steering mode switching, SAPP state machine |
| `opendbc_repo/opendbc/car/ford/fordcan.py` | CAN message builders (APA, LKA, stock) |
| `opendbc_repo/opendbc/car/ford/carstate.py` | SAPP handshake state parsing |
| `opendbc_repo/opendbc/car/ford/values.py` | SteeringMode enum, CarControllerParams |
| `opendbc_repo/opendbc/safety/modes/ford.h` | Panda safety (full passthrough) |
| `opendbc_repo/opendbc/safety/safety.h` | SAFETY_FORD treated as ALLOUTPUT |
| `system/sentry.py` | Sentry disabled |
| `system/manager/process_config.py` | Telemetry processes disabled |
