# Current validation boundary

The complete hardware, calibration, Brake_Test, Jetson, autonomy, and
production-readiness record is in `JETSON_PRODUCTION_HANDOVER.md`.

## Physical results supplied from the laptop bench

- Steering telemetry convention: left negative, right positive, centre near
  zero. Observed end ranges were approximately -47 to -48 degrees left and
  +76 to +78 degrees right.
- AS5047P hand-turn test: forward positive and reverse negative. One marked
  14T motor-sprocket turn was approximately one 16,384-count revolution.
- Nominal drivetrain geometry is 14T motor sprocket to 44T axle sprocket with
  an uncalibrated 11-inch tyre assumption. This is not a distance calibration.
- Thirty-second VCU stream: MOTION 49.54 Hz, CONTROL_FEEDBACK 20.004 Hz,
  HELLO 1 Hz, CRC errors 0, malformed frames 0, sequence gaps 0, validity 3,
  fault 0.
- Reboot epoch: PASS_DIAGNOSTIC; boot ID changed and count restarted safely.
- ROS bridge: live serial open and typed identity, wheel encoder, steering,
  state, and link status were observed.

## Known limitations

- Calibration ID remains `UNVERIFIED-LAPTOP-V1`.
- Tyre circumference and metres-per-count are not tape-calibrated.
- Wheelbase is not entered.
- The Jetson IMU is not wired or characterized.
- No ground-motion odometry accuracy, steering/yaw accuracy, slip threshold,
  stopping-distance, or full-autonomy test has passed.
- This bundle is NO-GO for autonomous vehicle motion.
