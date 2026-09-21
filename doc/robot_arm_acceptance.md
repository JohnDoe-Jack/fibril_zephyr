# Robot-arm acceptance status

Software evidence in this stack:

- C610 transport, feedback continuity, pair command, TTL, stop, and retry tests: PASS.
- RobStride codec, prepare/commit, cancellation epoch, and fake-CAN tests: PASS.
- Differential-screw DS-01..08 host behavior: PASS.
- Arm math/controller/enable ARM-01..09 and EN-01..06 host behavior: PASS.
- Input, neutral gate, B priority, mode, home, and tip-servo UI/SV behavior: PASS.
- RP2350-CAN `apps/robot_arm` compile/link and UF2 generation: PASS.
- Existing `apps/gamepad_servo` behavior: unchanged by the robot-arm app.

Hardware acceptance remains separate:

| ID | Check | Status |
|---|---|---|
| HIL-01 | USB CDC enumeration and stopped boot | NOT_RUN |
| HIL-02 | Shared 1 Mbit/s CAN and four feedback streams | NOT_RUN |
| HIL-03 | RS00 timeout readback on both axes | NOT_RUN |
| HIL-04 | RS00 stop/epoch recovery and bounded low motion | NOT_RUN |
| HIL-05 | C610 zero-current, signs, continuity, and low-current motion | NOT_RUN |
| HIL-06 | Differential lift/yaw directions and mechanical clearance | NOT_RUN |
| HIL-07 | GP17 tip PWM 1000/1210/1400 us and HOLD_LAST | NOT_RUN |
| HIL-08 | Simultaneous operation, CAN load, deadlines, and fault injection | NOT_RUN |

Do not mark hardware acceptance complete from these software results.
