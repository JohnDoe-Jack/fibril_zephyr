# PR1 C610 transport bring-up

This app exercises only the two physical C610/M2006 wrist axes (CAN IDs 1 and
2) on the Waveshare RP2350-CAN board. It does not implement differential-screw
kinematics.

Boot and the first three seconds are command-free. With actuator power and
termination checked, wait for fresh continuous feedback from both axes and
three fresh neutral gamepad frames. Press A once to start the RoboMaster
transport at zero current. After one second of continuous zero-current and
feedback validation, press Y to permit the low-current test.

Hold RT as a deadman and use the D-pad:

- Up/down: both axes `+0.25/-0.25 A`.
- Left/right: opposite `0.25 A` pair for sign checking.
- Release RT or the D-pad: zero current.
- B or link loss: latched stop and zero current.

The DTS cap is 0.5 A, the app cap is 0.25 A, and the command TTL is 30 ms.
Secure the mechanism and keep power removal accessible; zero current is not a
physical torque guarantee.
