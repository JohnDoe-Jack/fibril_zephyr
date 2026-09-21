# Planar robot-arm control

`CONFIG_ROBOT_ARM_CONTROL` ports the hardware-independent two-link arm stack.
It defines the shoulder/elbow `JointDrive` contract, forward and inverse
kinematics, continuous-angle tracking, transmission mapping, workspace checks,
motion profiles, the 500 Hz controller, and the fail-closed enable sequence.

The joint convention is important: `theta1` is the upper-arm angle and
`theta2` is the forearm absolute angle. The elbow joint angle is therefore
`theta2 - theta1`. Motor reduction and direction are applied exactly once by
the transmission layer; `JointDrive` positions are RS00 output-shaft radians.

The shipped profile uses 480 mm and 580 mm links, a positive elbow branch,
3:1/+1 shoulder transmission, and 1:1/-1 elbow transmission. Site geometry is
an initial software boundary only and has not been physically validated.

The controller is deliberately independent of Zephyr and motor drivers. One
execution context must own it. The enable helper is synchronous, so an
integration must call it only from that owner outside the control tick; it must
not run against the same Controller from a second thread. Enabling is
fail-closed: timeout protection is written and verified on both motors before
either is put in position mode, then measured positions are reseeded and the
controller enters Hold. Any partial failure returns both motors to the stopped
state.

Run the host regression with:

```sh
cmake -S tests/host/arm -B build/arm -G Ninja
cmake --build build/arm
ctest --test-dir build/arm --output-on-failure
```
