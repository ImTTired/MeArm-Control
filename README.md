
# MeArm Control

MeArm V3.0 robotic arm control project based on Arduino UNO.

## Hardware

- Arduino UNO
- MeArm robotic arm
- V3.0 dual-joystick expansion board
- Four servo motors
- External 5–6V servo power supply

## Pin Mapping

| Joint | Servo Pin | Joystick |
|---|---|---|
| Base | D9 | A0 |
| Left | D8 | A1 |
| Right | D7 | A3 |
| Claw | D6 | A2 |

## Implemented Features

- Independent control of four joints
- Real-time dual-joystick control
- Serial gripper commands (O/S)
- Speed adjustment (H/L)
- Three-servo coordinated control (x/y/z)
- Joystick/serial control mode switching
- Joint angle limits

## Serial Commands

Baud rate: 9600

| Command | Function |
|---|---|
| O | Open gripper |
| S | Close gripper |
| H | High-speed mode |
| L | Low-speed mode |
| J | Joystick mode |
| M | Serial mode |
| x90,y90,z90 | Set Base, Left and Right targets |

For V1.2, set Serial Monitor line ending to Newline.

## Safety

The current version uses conservative test limits:

- Base: 85–95 degrees
- Left: 85–95 degrees
- Right: 85–95 degrees
- Claw: 30–45 degrees

These limits are for initial hardware validation, not the complete working range.

Disconnect servo power before uploading new firmware. Verify mechanical clearance before restoring power.

## Development Status

- [x] Arduino environment setup
- [x] Joystick and button input tests
- [x] Individual servo movement tests
- [x] Joystick control
- [x] O/S/H/L serial communication
- [x] Three-servo serial control
- [ ] Full-range mechanical calibration
- [ ] Automatic pick-and-place
- [ ] Motion recording and playback
- [ ] Forward/inverse kinematics
- [ ] Drawing trajectory control

## Notes

This repository contains a student robotics assessment project.
Development and testing are ongoing.
