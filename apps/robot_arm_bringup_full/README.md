# Full robot-arm bring-up

This PR5 app builds the production integration path with a deliberately reduced hardware envelope:
RS00 IDs 1/2 at 1 rad/s and 1 A, C610/M2006 IDs 1/2 at 0.5 A, wrist lift at 5 mm/s,
wrist yaw in 5 degree steps, and arm tool jog at 20 mm/s. The tip servo is PWM0B on GP17,
the gamepad receiver is UART1 RX on GP21, and all four motors share the XL2515 CAN bus.

The first three seconds are command-free. The app then starts the C610 zero-current stream, releases
both RS00 axes, and proves zero output for another second. Release all controls for three fresh
gamepad frames, place the mechanism at the documented home, press **Y**, then press **A** to enable.

- Hold **RT** for motor motion. In the default mode, the left stick performs low-speed tool jog.
- Press **Menu** to select wrist mode; while holding RT, D-pad up/down jogs lift and left/right makes
  a 5 degree yaw step. Arm tool velocity is zero in this mode.
- Hold **LT** (without RT) and tap **LB/RB** to move the tip servo by 5 us. No PWM command is issued
  during boot.
- **B**, gamepad loss, RS00 controller fault, or wrist feedback loss removes motor output. Re-home and
  re-enable only after inspecting the cause.

Test one subsystem at a time: arm first, then wrist, then tip servo. Keep the mechanism supported and
do not use this app to discover hard stops.

