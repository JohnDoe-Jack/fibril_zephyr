# Robot arm application

This application integrates two RobStride RS00 joints (CAN IDs 1 and 2), two
C610/M2006 differential-wrist axes (IDs 1 and 2), the UART gamepad receiver,
and the GP17 tip servo on one RP2350-CAN board and one shared 1 Mbit/s CAN bus.

On boot every motor output is stopped and no servo pulse is started. Send three
fresh neutral gamepad frames, establish the hand-set safe home with Y while
stopped, then press A to prepare and enable both RS00 axes. B is level-sensitive
and always wins: it stops both motor families. A link timeout has the same motor
effect. MENU switches Hold/WristJog. In WristJog, D-pad up/down controls lift
and left/right steps yaw. LB/RB changes the tip pulse by 5 us per new press,
within 1000..1400 us; its first command starts PWM. Motor stop, input loss, and
CAN failure retain the last tip pulse (`HOLD_LAST`) and never auto-center it.

This software does not prove safe mechanics. Bring-up must be staged: flash and
boot with no commands, verify USB and input, verify all four feedback streams,
send zero wrist current, verify RS00 timeout/readback, then test bounded motion
with the mechanism secured and an accessible power/E-stop path.
