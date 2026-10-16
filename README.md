# MeArm Control

Arduino UNO-based control software for a MeArm V3.0 robotic arm, developed through joystick calibration, serial control, pose teaching, automatic pick-and-place, and synchronized motion planning.

**Current tested version:** `V1.5.1` · **Status:** Object A grasping verified; Objects B/C and dual-Arduino controller remain pending.

## 1. Hardware and pin mapping

- Arduino UNO with V3 dual-joystick expansion board
- MeArm V3.0, four hobby servos, modified gripper
- External regulated servo supply (with a shared ground where appropriate)

| Joint | Servo pin | Joystick analog pin | Role |
| --- | --- | --- | --- |
| Base | D9 | A0 | Rotates the base horizontally |
| Left | D8 | A1 | Moves the main arm |
| Right | D7 | A3 | Raises/lowers the front arm |
| Claw | D6 | A2 | Opens/closes the gripper |

**V1.5.1 configured joint-angle limits** (from the team's tested configuration): Base `0–180°`, Left `45–160°`, Right `0–180°`, Claw `30–100°`. The initial command pose is `(90, 90, 90, 30)`. These are software limits, not a guarantee that every combination is collision-free.

## 2. Project structure and program guide

Each Arduino sketch should reside in a folder with the same name as its `.ino` file. Include the entries below **only if the corresponding files are actually present** in the repository.

| Sketch / phase | Purpose | Main difference from previous phase |
| --- | --- | --- |
| `Joystick_Test` (experiment) | Read analog joystick axes and button states over Serial | Verified joystick inputs and pin wiring |
| `Servo_Test` (experiment) | Command one joint at a time | Verified servo pins, directions, and initial angles |
| `MeArm_Control_V1_1` | Four-joint joystick control and gripper/speed serial commands | Added `O`, `S`, `H`, `L` and per-joint software limits |
| `MeArm_Control_V1_2` | Coordinated serial joint positioning | Added `xNN,yNN,zNN`, joystick/serial modes `J` and `M` |
| `MeArm_Control_V1_3` | Pose teaching / inspection | Added `P` to print four commanded joint angles; adopted tested working-angle configuration |
| `MeArm_Control_V1_4` | Automated pick-and-place sequence for object A | Added `A` command, nine stored poses, auto-sequence state machine, and `!` stop |
| `MeArm_Control_V1_5` | Synchronized, smooth multi-joint movement | Replaced fixed 1° stepping with time-based quintic interpolation; updates about every 20 ms |
| **`MeArm_Control_V1_5_1`** | **Refined speed-limited automatic pick-and-place** | **Corrected quintic peak-speed calculation and introduced independent speed limits for each auto step** |

The naming in this table reflects the development stages; the repository may also contain earlier experiments with different filenames.

## 3. Version history and implementation details

### V1.1 — Joystick and basic serial commands

- Control four joints independently using the dual joysticks.
- Read single-character commands: `O` opens the gripper, `S` closes it, `H` selects higher speed and `L` lower speed.
- Protect the mechanism with conservative software limits during initial testing.
- Movement implemented as small, regularly timed changes to the servo's commanded angle.

**Limitation:** The motion was not synchronized across joints, and speed changes were coarse.

### V1.2 — Three-joint serial positioning

- Added `x90,y90,z90`-style serial commands (the values specify **Base, Left, Right** in degrees).
- Added `J` (joystick mode) and `M` (serial mode) to avoid competing inputs.
- Retained existing joystick and single-character serial functions.

**Limitation:** Joint movement still used fixed-size angular steps; a short-travel joint could finish much earlier than a long-travel joint.

### V1.3 — Pose teaching

- Added `P`: print the four **commanded** joint angles in `Base,Left,Right,Claw` order.
- Used the output to record reproducible pickup poses instead of guessing angles.
- Expanded the earlier narrow test limits to the team's tested configuration.

**Note:** `P` reports software-commanded positions, not encoder measurements of the actual shaft angles.

### V1.4 — Object A automatic pickup and placement

- Added `A` to run a stored sequence: **APPROACH → PICK → GRASP → LIFT → TRANSFER → PLACE → RELEASE → RETRACT → HOME**.
- Added a non-blocking automatic step controller with pauses at each pose.
- Added `!` as a software stop command; maintained serial and joystick control.
- Object A's approach, pickup and grip poses were taught on the real mechanism; transfer and placement poses were subsequently tested as part of the A sequence.

**Limitation:** Fixed `1°` changes at a common interval meant joints traveled at the same nominal rate, regardless of the distances each joint needed to move. Different joints therefore stopped at different times; starting and stopping could look jerky.

### V1.5 — Synchronized quintic trajectory planning

- Replaced fixed 1° stepping with a **time-based, four-joint interpolated motion**.
- All joints in one move use the same normalized time `t`, so their *commanded trajectories* start and finish together.
- Applied quintic smoothstep interpolation: `s(t) = 10t³ − 15t⁴ + 6t⁵`.
- Updated servo target pulses approximately every `20 ms`; used fractional-degree internal positions.
- Introduced a single global `motionSpeed = 15.0f`, interpreted by the code as the average speed (°/s) of the joint with the largest movement.

**Remaining issue:** Even with a smooth velocity curve, the midpoint velocity is substantially higher than the average. The pickup-to-lift motion visibly accelerated too much.

### V1.5.1 — Corrected peak velocity and step-dependent speeds

**This version is a targeted update to the tested V1.5 implementation, not a rewrite of the motion engine.** It retains the same servo/joystick pin mappings, A sequence poses, quintic interpolation, and existing serial commands.

#### A. Correct interpretation of speed

The quintic function's **maximum derivative is 1.875**. V1.5 computed the segment duration as:

```cpp
float durationSeconds = maxDistance / motionSpeed;
```

As a result, a `15°/s` *average* commanded joint speed could reach a **28.125°/s peak** during the segment.

V1.5.1 instead treats `motionSpeed` as the allowable **peak commanded joint speed**, with the duration calculated as:

```cpp
float selectedSpeed =
    (peakSpeed > 0.0f) ? peakSpeed : motionSpeed;
float durationSeconds =
    1.875f * maxDistance / selectedSpeed;
```

The default peak speed was changed from the former `15°/s` average-speed setting to a `12°/s` peak-speed setting. Manual/serial `H` now sets `16°/s`, while `L` sets `8°/s`.

#### B. Different limits for different actions

V1.5.1 adds a nine-element `autoStepSpeed[]` table aligned with `sequenceA[]`:

| Action | Peak command speed (°/s) |
| --- | ---: |
| APPROACH | 12 |
| PICK | 8 |
| GRASP | 8 |
| **LIFT** | **8** |
| TRANSFER | 12 |
| PLACE | 8 |
| RELEASE | 8 |
| RETRACT | 12 |
| HOME | 12 |

The automatic sequence passes the speed for each step into `startPose()` → `startMotion()`. Ordinary joystick, `O/S`, and `x/y/z` commands still use the global `motionSpeed`, so the previous interfaces are preserved. **`H/L` affect ordinary/manual/serial movements, not the independently configured automatic A step speeds.**

#### C. Worked example — Why the lift became smoother

The **GRASP → LIFT** transition changes the Right joint from `43°` to `86°`, a **43° movement**:

| Configuration | Planned segment duration | Planned peak commanded speed |
| --- | ---: | ---: |
| V1.5 (`15°/s` average) | ~2.87 s | ~28.13°/s |
| V1.5.1 default (`12°/s` peak) | ~6.72 s | 12°/s |
| **V1.5.1 LIFT step (`8°/s` peak)** | **~10.08 s** | **8°/s** |

These are **software trajectory calculations**, not measured servo velocities. The smoother motion of V1.5.1 was confirmed by a real-world run of the arm.

#### D. What did *not* change

- Hardware mapping, working-angle configuration and initial pose
- Existing object A pose sequence and automatic step order
- Smoothstep polynomial and synchronized interpolation
- `A`, `!`, `P`, `J`, `M`, `O`, `S`, `H`, `L`, and `x/y/z` command interfaces

## 4. Serial command reference (V1.5.1)

Use **9600 baud** and **Newline** as the Serial Monitor line ending.

| Command | Description |
| --- | --- |
| `J` | Joystick mode |
| `M` | Serial positioning mode |
| `P` | Print commanded pose (`Base,Left,Right,Claw`) |
| `O` | Open gripper (`30°` target) |
| `S` | Close gripper (`84°` target) |
| `H` | Set ordinary-motion peak command speed to `16°/s` |
| `L` | Set ordinary-motion peak command speed to `8°/s` |
| `x90,y90,z90` | Command Base, Left and Right angles; claw target is not changed |
| `A` | Run automatic object A sequence |
| `!` | Stop updating the active planned motion (software stop; not a hardware emergency stop) |

The `A` sequence uses its own per-step speed table, independent of the currently selected `H/L` speed. The configured angles describe servo setpoints, not real-time measured positions.

## 5. Object A: taught trajectory

The active V1.5.1 sequence stores joint poses in the order `Base,Left,Right,Claw`:

| Step | Base | Left | Right | Claw |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 60 | 130 | 86 | 30 |
| PICK | 60 | 130 | 43 | 30 |
| GRASP | 60 | 130 | 43 | 84 |
| LIFT | 60 | 130 | 86 | 84 |
| TRANSFER | 100 | 130 | 86 | 84 |
| PLACE | 100 | 130 | 55 | 84 |
| RELEASE | 100 | 130 | 55 | 30 |
| RETRACT | 100 | 130 | 86 | 30 |
| HOME | 90 | 90 | 90 | 30 |

These poses are specific to the assembled arm and the marked work area; moving the robot base, changing the assembly, or reconnecting servos incorrectly can invalidate the taught trajectory.

## 6. Verification and current status

| Feature | Status |
| --- | --- |
| Four-servo joystick control | Tested |
| O/S/H/L serial operation | Tested in earlier versions |
| Three-joint serial positioning | Tested |
| `P` pose output | Tested |
| Object A grasping | **Successfully demonstrated on hardware** |
| V1.5.1 smooth lift behavior | **Improved and observed to be smooth on hardware** |
| Object B / C automatic sequences | Not yet implemented or verified |
| Task 3: dual-Arduino buttons, recording and playback | Not yet implemented |

## 7. Running and safety

1. Open a sketch in Arduino IDE, select **Arduino Uno** (`arduino:avr:uno`) and the connected COM port.
2. Compile and upload with the independent servo power supply disconnected.
3. Recheck D6–D9 signal connections, correct signal/power/ground orientation, the common ground, and safe mechanical startup posture before applying servo power.
4. Use Serial Monitor at **9600 baud**, **Newline**. Verify small movements first; perform an unloaded A sequence before grasping a real object after wiring or mechanical changes.
5. Keep an accessible way to disconnect servo power. The `!` command is a **software motion stop**, not a substitute for a physical emergency power cutoff.

## 8. Planned development

- [x] Task 1: manual, joystick, and serial control
- [x] Task 2: automatic grasp of object A
- [x] Task 2: synchronized motion and speed optimization
- [ ] Task 2: object B and C automatic pick-and-place demonstrations
- [ ] Task 3: second Arduino + four-button wired control
- [ ] Task 3: repeated pick, motion recording, playback, HOME
- [ ] Subsequent kinematics and trajectory-drawing tasks

> **Development note:** Old versions are preserved for reproducibility. Actual Git commits record when the files are committed; individual filenames describe development stages rather than implying separate historic commits.
