# PR2 RobStride RS00 bring-up

This app exercises only the physical shoulder RS00 (ID 1) and elbow RS00
(ID 2) on the shared 1 Mbit/s CAN wiring. It does not apply arm kinematics.

The first three seconds are command-free. After three fresh neutral gamepad
frames:

1. Press A to send a new-epoch stop and obtain fresh feedback from both axes.
2. Press Y to prepare CSP on both axes without enabling torque. This writes the
   100 ms timeout, selects mode 5, reads the present mechanical position, and
   seeds `loc_ref`.
3. Press X to commit both axes and hold the measured positions.
4. Hold RT and use up/down for shoulder or left/right for elbow. The target is
   only `±0.02 rad` from the captured position, with 1 rad/s and 1 A limits.

B, link loss, stale/invalid feedback, a partial prepare/commit, or a command
failure stops both axes and advances the operation epoch. Keep the mechanism
supported: holding the current encoder position can still produce torque.
