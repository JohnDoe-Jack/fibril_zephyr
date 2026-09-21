# Robot-arm staged bring-up

Do not begin by rotating a C610 or enabling an RS00. The software build and
host tests do not establish correct wiring, mechanics, directions, or limits.

1. Build `apps/robot_arm` for `waveshare_rp2350_can/rp2350a/m33`. Inspect the
   produced `zephyr.uf2`; do not reuse a generic README path from another app.
2. Disconnect actuator power, flash the UF2, and confirm the USB CDC product
   `RP2350 Robot Arm` appears. Confirm the application reaches its stopped
   readiness message and no tip-servo pulse is emitted.
3. Verify UART gamepad reception. Send three distinct neutral frames. Confirm
   B remains level-sensitive and that disconnect produces the stop state.
4. With CAN termination and an accessible power/E-stop path checked, energize
   the motor supply but send no motion command. Confirm RS00 feedback IDs 1/2
   and C610 feedback IDs 1/2 independently. Reject stale or discontinuous data.
5. Exercise stop only: verify both RS00 axes disable and the C610 pair receives
   zero current. The tip servo must retain its last pulse; PWM stop is not a
   power cut or guaranteed torque release.
6. Verify the RS00 100 ms communication timeout, mode readback, mechanical
   position seed, 8.5 rad/s limit, and 6 A limit on both axes before committing
   either enable. A failure on one axis must stop both and advance the epoch.
7. With the wrist mechanically secured, command zero current first. Confirm
   encoder continuity, direction signs, and the 36:1 conversion. Then test a
   tightly bounded current below the 4 A software limit.
8. Establish the safe physical home while stopped. Test shoulder, elbow, lift,
   yaw, and the GP17 tip servo one at a time at low limits before simultaneous
   operation. Keep HIL acceptance NOT_RUN until every observation is recorded.

Fault recovery is always stop → remove the cause → regain fresh continuous
feedback → re-establish home if power/mechanics changed → repeat timeout
verification and seeding → enable. Never reuse an old operation epoch.
