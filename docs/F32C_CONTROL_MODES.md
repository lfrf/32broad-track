# F32C Vision Control Modes

The vision protocol still uses the existing `dx` and `dy` fields. In the current
camera firmware those fields carry `ex10` and `ey10`: target-plane errors in
0.1 mm units. The packet layout is unchanged.

## Select a mode

Edit one line in `User/f32c_gimbal.h`:

```c
#define F32C_TRACK_CONTROL_MODE F32C_TRACK_MODE_EX10_SPEED_PD
```

- `F32C_TRACK_MODE_EX10_SPEED_PD`: direct speed PD outer loop. This is the new
  default and is intended to approach a stationary target faster.
- `F32C_TRACK_MODE_POSITION_VELOCITY`: the previous position target plus
  velocity feed-forward controller. Use this as the immediate rollback mode.

No camera-side protocol change is required when switching these two modes.

## Speed PD behavior

The proportional term converts target-plane error into motor speed. The
derivative term damps a rapidly changing error. Input filtering, independent
axis dead zones, derivative limits, speed limits, and per-update slew limits
prevent a single noisy frame from commanding a large speed step.

If no new valid vision sample arrives for 100 ms, both target speeds are set to
zero and the PD history is reset. A stale error is never driven indefinitely.

## First test

1. Disable the laser and support the gimbal so a wrong command cannot damage it.
2. Put the target to the right of image center. Confirm yaw moves toward it. Stop
   immediately and correct the direction macro if it moves away.
3. Repeat for pitch with the target above and below center.
4. Test a small error near the target and confirm both axes settle inside the
   dead zone without hunting.
5. Test a larger error and confirm speed rises smoothly and remains limited.
6. Cover the camera or disconnect vision input. Both motors must stop within
   about 100 ms.

## Tuning order

Keep the initial values conservative and change only one group at a time:

1. Set the dead zones from the measured stationary `ex10` and `ey10` noise.
2. Increase `*_KP_NUM` only until approach speed is adequate.
3. Increase the speed limits only if the proportional command reaches them.
4. Increase `*_KD_NUM` if the gimbal overshoots; reduce it if noise causes buzz.
5. Increase the slew limits only if acceleration still feels unnecessarily slow.

The pre-change remote checkpoint is commit `6ec9cfa`.
