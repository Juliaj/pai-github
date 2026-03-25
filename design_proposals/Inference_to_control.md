# Preliminary Thinking: Inference to ros2_control

Date: March 2026

---

## 1. Problem Statement

Neural policy inference runs at 10-50Hz. Robot hardware requires 200Hz-2kHz for stable control. Currently, ros2_control has no controller that bridges this gap.

Joint Trajectory Controller (JTC) interpolates between waypoints but assumes pre-planned trajectories, not continuous policy output. Used with streaming inference, it exhibits:

- Jerky motion from position-only cubic interpolation
- Hard discontinuities at chunk boundaries
- No filtering of unphysical policy outputs
- No mechanism to absorb inference timing jitter

This docuemnt outlines the idea of a pair of chainable ros2_control controllers that upsample, blend, and gate neural policy output for hardware-rate command generation. Target platform is SO-ARM101, with validation planned in both simulation and hardware.

---

## 2. Background

### 2.1 JTC vs Bridge

| Capability | JTC | Bridge (To be developed) |
|---|---|---|
| Interpolation | Cubic spline | Quintic / MJT |
| Upsampling to control rate | Position-only | Position + velocity continuity |
| Streaming inference interface | No | Yes |
| Look-ahead jitter buffer | No | Yes |
| Chunk boundary blending | No | Yes |
| Jerk monitoring and gating | No | Yes |

JTC accepts a complete trajectory upfront with no mechanism for mid-execution chunk replacement, jitter-aware buffering, or acceleration/jerk limiting.

### 2.2 Three Frequency Layers

The design keeps three frequency domains separate:

| Layer | Rate | Owner |
|---|---|---|
| Policy inference | 10-20Hz | LeRobot async inference |
| Waypoint reference | 50Hz | PolicyController -> BridgeController |
| Hardware command | 200Hz | BridgeController -> servo bus |

LeRobot's async inference ensures chunk N+1 is ready before chunk N is exhausted (top gap). The bridge upsamples 50Hz waypoints to 200Hz hardware commands (bottom gap). These responsibilities do not overlap.

### 2.3 Relationship to Action Chunking

Action chunking (ACT-style) provides 1s of look-ahead (H=50 steps at 50Hz) as a natural buffer. It also introduces problems the bridge must handle:

- Chunk boundary discontinuities: last state of chunk N and first state of chunk N+1 are not guaranteed C1 or C2 continuous
- Temporal ensembling changes smoothness properties of the reference trajectory
- Even at 50Hz waypoints, actuators need 200Hz commands; cubic interpolation still produces jerk at knots
- Variable replan timing (every 5-10 steps, not at chunk exhaustion) requires managed handoff

---

## 3. System Architecture

```text
ACT / Diffusion / VLA policy  (10-20Hz)
        |
        |  [LeRobot async inference - chunk queue]
        v
PolicyController               [ros2_control chainable controller]
  - ONNX runtime on non-RT thread
  - lock-free ring buffer, N=4 chunks
  - signals RT thread when queue below threshold
  - assembles observation from joint states + camera
        |
        |  [ActionChunk reference interface - 50Hz waypoints]
        v
BridgeController               [ros2_control chainable controller]
  - quintic MJT upsampling: 50Hz -> 200Hz
  - chunk boundary blending: 20% tail overlap
  - velocity estimation: Savitzky-Golay if policy is position-only
  - jerk monitor: scale trajectory, halt if threshold exceeded
  - publishes diagnostics: /so101/bridge/jerk_violation
        |
        |  [hardware command interface - 200Hz]
        v
        +-- mujoco_ros2_control    (simulation)
        +-- Feetech STS3215        (SO-ARM101 hardware)
```

### 3.1 Frequencies for SO-ARM101

| Parameter | Value | Rationale |
|---|---|---|
| Control rate | 200Hz | Feetech STS3215 at 1Mbaud, 6 joints x ~0.5ms round-trip = ~3ms minimum; 200Hz within bus limits |
| Inference rate | 20Hz | ACT on laptop GPU (RTX 3060 class) runs 15-25Hz; 50ms per chunk gives 2.5 waypoints of look-ahead |
| Chunk size | 50 steps | 1 second of trajectory at 50Hz internal density; 20x replan period as buffer |
| Upsample ratio | 4x | 50Hz -> 200Hz; quintic spline synthesizes 4 x 5ms ticks per 20ms waypoint interval |
| LeRobot threshold | 10 steps | `chunk_size_threshold=10`; triggers new inference when 200ms of actions remain in queue |

---

## 4. Validation Strategy

### 4.1 Simulation -- mujoco_ros2_control

Primary quantitative validation. Runs identical controller code to hardware with ground-truth state access.

- Frequency verification: controller runs at 200Hz, waypoints arrive at 50Hz, no RT thread blocking (update() p99 < 2ms)
- Jitter injection: artificial inference delays (random sleep 0-30ms before chunk emit), verify buffer absorbs without trajectory stall
- Chunk boundary artifacts: joint acceleration spikes at chunk handoff visible in MuJoCo logs; compare pre/post bridge
- Jerk metric comparison: position-only vs position+velocity policy through same bridge; ground-truth jerk from MuJoCo internal state
- Validation script: 30-second episode, plots joint position/velocity/jerk, prints PASS if no jerk spikes and queue never empty

### 4.2 Hardware -- SO-ARM101

Qualitative demonstration and sim-to-hardware transfer verification.

What hardware validates:

- Smooth vs jerky execution visible and audible on Feetech servos
- Chunk boundary shudder before/after bridge
- Sim-to-hardware transfer of controller behaviour
- Real inference pipeline (ONNX on laptop GPU) end-to-end

What hardware does not validate:

- Absolute jerk threshold correspondence (no torque sensing on STS3215)
- Full 1kHz control rate (servo bus caps at ~200Hz with 6 joints)
- Rigorous safety claims (software monitor only, non-RT OS)

Hardware protocol:

- 5 episodes of block color sorting task (3 colored blocks -> matching target zones, ~15s per episode)
- Record: task success, max jerk observed, jitter monitor trigger count, chunk queue minimum depth, end-effector path smoothness (FK on joint states)
- Pass criterion: >=3/5 episodes succeed, no jerk monitor triggers on successful episodes

### 4.3 Safety Scope

The bridge mitigates inference pipeline artifacts -- jitter, chunk discontinuities, unphysical commands. It is not a safety system per ISO 13849 / ISO/TS 15066.

| Layer | Function | Status |
|---|---|---|
| 1. Mission / task | Should the robot be doing this at all? | Open research problem |
| 2. Runtime policy monitor | Is the policy behaving correctly? | Uncertainty quantification (near-term); reachability verification (research) |
| 3. Physical command safety | Is this command physically safe? | This proposal |
| 4. Hardware safety | Is this within actuator limits? | Firmware (Franka/UR); weak on Feetech |

The bridge is layer 3. It reduces risk of harm from inference pipeline artifacts. It does not address whether the policy itself is behaving correctly -- that requires layers 1 and 2.

---
