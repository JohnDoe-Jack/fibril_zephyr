# Differential-screw wrist library

`CONFIG_DIFFERENTIAL_SCREW` provides a hardware-independent C++ library for
the two C610/M2006 axes in the differential-screw wrist. It has no dependency
on Zephyr device APIs, CAN, RobStride, or the planar-arm library.

The public API is under `include/differential_screw`. Callers convert each
motor's extended encoder count and rotor speed to nut radians with
`nutRadFromCounts()` and `nutRadPerSecFromRpm()`, seed `Wrist` from both current
positions, then call `step()` at the command period. Until seeding succeeds,
`step()` always returns zero current.

The default configuration preserves the source mechanism: 100 mm lead,
36:1 total ratio, 8192 rotor counts/revolution, 4 A current limit, 2 mm/0.1 rad
control leashes, and independent common/differential damping. Yaw commands are
kept continuous across full revolutions. Stall protection detects sustained
near-limit current with low nut velocity and applies a finite number of relief
cycles before remaining at the 2 A hold level.

There is intentionally no finite lift-travel limit because none was supplied
or verified for the mechanism. The integrated application must therefore keep
the wrist disabled until feedback continuity is valid and establish safe
mechanical travel during staged hardware bring-up.

Run the host regression with:

```sh
cmake -S tests/host/differential_screw -B build/differential_screw -G Ninja
cmake --build build/differential_screw
ctest --test-dir build/differential_screw --output-on-failure
```
