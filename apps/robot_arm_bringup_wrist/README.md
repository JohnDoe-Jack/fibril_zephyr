# Differential-screw wrist bring-up

This app exercises PR3's differential-screw control with the physical wrist pair: C610/M2006 ID 1
is the right nut and ID 2 is the left nut on the shared 1 Mbit/s CAN bus. Gamepad input is received
on GP21 and logs use USB CDC.

The fixture must hold arm joint `theta2` at zero. This app intentionally tests wrist-relative yaw;
full arm-angle compensation belongs to the later integrated bring-up.

1. Power the controller with motor power disabled and confirm the first three seconds are command-free.
2. Enable motor power, confirm continuous feedback for both motors, release all controls three times,
   then press **A**. The app starts the C610 transport and proves zero current for one second.
3. Put the wrist in its known yaw-home pose and press **Y** to capture feedback and set yaw zero.
4. Press **X** to enter the 0.5 A limited control stage.
5. Hold **RT** as the deadman. D-pad up/down jogs lift at 5 mm/s; D-pad left/right makes one
   edge-triggered 5 degree yaw step. Releasing RT holds the measured target rather than continuing a jog.
6. Press **B**, disconnect the gamepad, or interrupt either feedback stream to command zero and forget
   the captured reference. Return to step 2 after inspecting the fault.

Do not use this app to find hard stops. Verify nut directions with the mechanism unloaded first.

