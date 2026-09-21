# Robot-arm acceptance bring-up

This PR6 app uses the same reduced-output runtime and wiring as
`robot_arm_bringup_full`, then adds once-per-second acceptance telemetry and a deliberately safe
fault-injection gesture. Follow the full app's command-free, zero-output, home, and enable sequence.

The USB CDC diagnostic line records:

- 2 ms control-work deadline misses and the maximum observed work time;
- RoboMaster operation epoch, TX errors, command timeouts, and both continuity epochs;
- RobStride operation epoch, TX errors, rejected commands, and both feedback ages;
- enable state and the number of injected stops.

While the mechanism is supported, hold **View** and tap **X** to inject a software-requested safe
stop. The app releases both RS00 axes, writes zero to the C610 pair, increments the counter, and
requires the normal home/enable sequence before resuming. Hold **View** and tap **Menu** to clear only
the timing counters; it does not clear motor faults or enable output.

Acceptance order: run each PR1--PR4 app first, run the PR5 full app one subsystem at a time, then use
this image for mixed arm/wrist operation, safe-stop recovery, gamepad disconnect, CAN power-cycle,
and continuity-epoch observation. A successful build is not a hardware pass; record the CDC log and
observed mechanism response for each case.

