# Two-link arm bring-up

This app exercises PR4's real arm controller with the shoulder RS00 at CAN ID 1 and elbow RS00 at
CAN ID 2. The wrist motors and tip servo are deliberately excluded. Gamepad input is on GP21 and
logs use USB CDC.

1. Keep the arm supported and motor power disabled. Confirm that boot emits no motor command for
   three seconds.
2. Enable motor power, release all controls three times, then press **A**. This sends a new-epoch
   stop and discovers fresh, continuous feedback from both joints.
3. Mechanically place the arm at the documented home pose (upper arm +X, forearm +Y). While the
   controller is released, press **Y** to accept the transmission/home reference.
4. Press **X** to prepare both drives with a 100 ms CAN timeout, 1 rad/s speed limit, and 1 A current
   limit, commit both, seed from measured positions, and hold without a target jump.
5. Hold **RT** as the deadman. D-pad up/down jogs shoulder and left/right jogs elbow at at most
   0.10 rad/s. **Menu** toggles to tool mode, where the left stick jogs the tip at at most 20 mm/s.
6. Press **B**, disconnect the gamepad, lose feedback, or trip a controller fault to release both
   drives. A new A/Y/X sequence is required before motion resumes.

The Y action declares the configured mechanical home; it does not discover hard stops. Confirm the
documented pose and transmission directions before pressing Y.

