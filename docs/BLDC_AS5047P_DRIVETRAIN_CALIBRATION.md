# AutoKart BLDC encoder and rear-drivetrain calibration

Date: 22 October 2026  
Status: Mechanical layout recorded; nominal conversion available; physical calibration pending

## 1. Purpose

This document records the mechanical arrangement of the AS5047P BLDC encoder
and defines how its readings will be converted into sprocket speed, rear-axle
speed, estimated vehicle speed, and estimated distance.

The AS5047P is not mounted directly on the rear axle. Its magnet is fixed at
the centre of the 14-tooth BLDC output sprocket. The encoder PCB is stationary,
centred on the same axis, perpendicular to the sprocket face, and faces the
magnet.

```text
                    stationary AS5047P PCB
                             │
                             │ sensor faces magnet
                             ▼
BLDC output ── [14T sprocket + centred magnet]
                         ║
                         ║  chain
                         ║  stated length: 72.5 cm
                         ║
rear axle ───────── [44T sprocket] ───────── rear tyre
                                              11 × 7.10 - 5
                                              BKT SM 68
```

## 2. Recorded mechanical data

| Item | Recorded value | Validation state |
|---|---:|---|
| Encoder | AS5047P, 14-bit magnetic rotary encoder | SPI operation bench-tested |
| Encoder magnet location | Centre of 14T BLDC sprocket | Mounting described; runout/gap not measured |
| Encoder orientation | Stationary PCB perpendicular to sprocket and facing magnet | Alignment described; not measured |
| Driving sprocket | 14 teeth | Reported value; physically count/mark once |
| Driven sprocket | 44 teeth | Reported value; physically count/mark once |
| Chain length | 72.5 cm | Meaning, pitch and link count not yet recorded |
| Driven member | 44T sprocket rigidly connected to rear axle | Confirm key/coupling and absence of relative slip |
| Rear tyre | `11 × 7.10 - 5`, model BKT SM 68 | Model/size identified; loaded circumference not measured |

The tyre designation nominally describes approximately 11 inch overall
diameter, 7.10 inch section width, and a 5 inch rim. BKT lists SM 68 as a
go-kart tyre in this size. The 11 inch value must not be treated as the exact
loaded rolling diameter.

Manufacturer reference:
[BKT SM 68 go-kart tyre](https://commercial.bkt-tires.com/pattern/sm-68-gk/).

## 3. Nominal drivetrain calculations

Assuming the 14T sprocket drives the 44T axle sprocket without another
reduction stage:

```text
motor-sprocket revolutions per axle revolution = 44 / 14
                                                = 3.142857

axle revolutions per motor-sprocket revolution = 14 / 44
                                                = 0.318182
```

Therefore:

- one complete 14T sprocket revolution produces approximately `0.318182`
  rear-axle revolutions;
- one rear-axle revolution requires approximately `3.142857` revolutions of
  the encoder magnet/14T sprocket;
- axle RPM is `encoder_sprocket_rpm × 14 / 44`;
- encoder sprocket RPM is `axle_rpm × 44 / 14`.

Using the nominal 11 inch tyre diameter only for preliminary calculations:

```text
nominal tyre diameter       = 11 in × 0.0254 = 0.2794 m
nominal circumference       = pi × 0.2794    = 0.877761 m
nominal travel / encoder rev = 0.877761 × 14 / 44
                              = 0.279288 m
```

With 14-bit SPI angle samples and `16384` ideal counts per full encoder
revolution:

```text
ideal encoder counts per axle revolution = 16384 × 44 / 14
                                          = 51492.57 counts

nominal counts per metre = 16384 / 0.279288
                         = 58663.55 counts/m
```

These count values are theoretical. Firmware must use the measured encoder
scale and measured loaded tyre circumference after calibration.

### Nominal speed reference

| Vehicle speed | Rear-axle RPM | 14T sprocket / encoder RPM |
|---:|---:|---:|
| 5 km/h | 94.9 | 298.4 |
| 10 km/h | 189.9 | 596.8 |
| 15 km/h | 284.8 | 895.1 |
| 20 km/h | 379.8 | 1193.5 |
| 25 km/h | 474.7 | 1491.9 |
| 30 km/h | 569.6 | 1790.3 |

## 4. Firmware conversion

For unwrapped AS5047P count change `delta_counts`, calibrated counts per
encoder revolution `counts_per_encoder_rev`, sample duration `delta_t_s`, and
measured loaded rolling circumference `wheel_circumference_m`:

```text
encoder_revolutions = delta_counts / counts_per_encoder_rev
axle_revolutions    = encoder_revolutions × 14 / 44

distance_m = axle_revolutions × wheel_circumference_m
speed_mps  = distance_m / delta_t_s

encoder_rpm = encoder_revolutions / delta_t_s × 60
axle_rpm    = encoder_rpm × 14 / 44
```

The sign must be calibrated so the chosen forward direction produces positive
RPM and distance. Do not silently reverse the sign without recording the
physical rotation used during calibration.

The encoder measures the 14T sprocket. It cannot independently detect a
broken chain, a slipping/keyless sprocket, tyre slip, or loss of coupling
between the 44T sprocket and rear axle.

## 5. Data still required

The supplied data is enough for nominal gearing calculations, but the
following measurements are required before speed or distance can be trusted:

1. **Magnet type:** confirm a correctly sized, diametrically magnetized magnet.
2. **Magnet retention:** use a mechanically secure, RPM-rated installation;
   adhesive alone must not be assumed safe next to an exposed chain drive.
3. **Alignment:** record axial sensor-to-magnet air gap, lateral offset and
   magnet/sprocket runout. Confirm `MAGNET OK` for a complete revolution at
   rest and at the maximum intended RPM.
4. **One-revolution encoder scale:** mark the 14T sprocket, rotate it exactly
   one revolution, and confirm one AS5047P angle wrap with no parity,
   diagnostic or missing-sample errors.
5. **Rotation direction:** record raw count direction for forward and reverse.
6. **Sprocket verification:** physically count and mark both sprockets as 14T
   and 44T.
7. **Chain details:** record chain pitch/type, total link count, sprocket centre
   distance, tension/slack and how the stated 72.5 cm was measured. Chain
   length is not part of the speed ratio, but is required to validate the
   mechanical installation.
8. **Coupling:** confirm the 44T sprocket is rigidly keyed/fixed to the axle and
   cannot rotate independently.
9. **Tyre pressure and load:** record operating cold pressure and representative
   kart/rider load during circumference calibration.
10. **Loaded rolling circumference:** mark the tyre and ground, roll the loaded
    kart straight for at least five rear-wheel revolutions, measure total
    distance, and divide by the exact revolution count. Repeat three times.
11. **Distance validation:** compare encoder-estimated distance with a measured
    straight-line distance of at least 10 m at low speed.
12. **Maximum speed validation:** confirm the maximum encoder RPM remains
    within the selected SPI sample rate and filtering limits.

## 6. Calibration procedure

### A. Power-off mechanical inspection

- Isolate traction power and prevent unexpected BLDC motion.
- Confirm 14T and 44T tooth counts.
- Inspect sprocket keys, axle coupling, chain alignment, tension and guarding.
- Measure and record magnet diameter, thickness, air gap and lateral offset.
- Confirm the magnet cannot detach at maximum sprocket RPM.

### B. Encoder direction and one-revolution test

- Turn the 14T sprocket slowly by hand through exactly one marked revolution.
- Record starting count, unwrapped count change and ending count.
- Verify one complete revolution corresponds to approximately 16384 raw
  14-bit counts when using SPI angle unwrapping.
- Repeat clockwise and counter-clockwise.
- Require `MAGNET OK`, valid parity and no unexplained jumps throughout.

### C. Static gearing verification

- Mark both sprockets.
- Rotate the 44T axle sprocket exactly one revolution.
- Verify the 14T encoder sprocket rotates approximately `44/14 = 3.142857`
  revolutions.
- Repeat in the opposite direction and check backlash/slack behavior.

### D. Loaded tyre calibration

- Inflate the rear tyre to the selected operating pressure.
- Apply representative kart load.
- Mark the tyre at the ground contact point.
- Roll straight for at least five complete tyre revolutions.
- Calculate `measured_distance / revolutions` for each of three trials.
- Use the mean as `wheel_circumference_m`; record the trial spread as
  calibration uncertainty.

### E. Low-speed end-to-end validation

- Run wheels off the ground first, then conduct a restrained low-speed test.
- Compare encoder direction and RPM with an independent tachometer where
  available.
- Compare estimated distance against at least 10 m of physically measured
  travel.
- Inspect raw and filtered data for parity errors, jumps, stale samples and
  implausible acceleration.

## 7. Proposed acceptance criteria

These are initial engineering targets and must be approved before becoming a
vehicle safety requirement:

- `MAGNET OK` throughout the complete speed range.
- No AS5047P parity or diagnostic errors during a continuous test run.
- One marked sprocket revolution agrees with the calibrated count scale.
- Forward/reverse signs are consistent and documented.
- Estimated distance error is no greater than 2% over a measured 10 m run.
- Filtered stationary RPM returns close to zero without hiding stale data.
- No contact between sensor PCB, magnet, sprocket or chain at any speed.
- Magnet, sensor mount and chain guard remain secure after the test.

## 8. Current conclusion

The available information is sufficient to implement a **preliminary nominal
conversion** using a `44/14` reduction. It is **not sufficient for final RPM,
speed, distance, odometry, braking-distance or autonomous-control validation**.

The minimum missing values are:

- measured loaded rear-tyre circumference;
- verified encoder counts per marked 14T sprocket revolution;
- forward/reverse count sign;
- encoder/magnet air gap, alignment and maximum-RPM magnetic health;
- chain pitch/link count and verified sprocket-to-axle coupling;
- independent measured-distance and RPM comparison.

Until those values are measured, firmware must label speed and distance as
**estimated / uncalibrated**.
