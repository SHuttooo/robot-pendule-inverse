# ESP32 firmware

**English** · [Français](README.fr.md)

A single Arduino sketch, `robot_balancier/robot_balancier.ino`, and the
generated header `robot_balancier/politique.h`. Code comments are in French.

| | |
|---|---|
| board | ESP32 Dev Module (`esp32:esp32:esp32`) |
| core | Espressif esp32 3.3 (2.x compatible through the `TIMER_*` macros) |
| size | 364 kB flash (27%), 71 kB RAM (21%) |
| libraries | nothing outside the core: `Wire`, `Preferences` |

```
arduino-cli compile --fqbn esp32:esp32:esp32 robot_balancier
```

**Before flashing:** empty `tools/cmd.txt` (a leftover command is sent on
reboot), and put the robot down: flashing cuts power to the board for eight
seconds.

---

## Two controllers for the same robot

**The dual PID** (command `1`), tuned by hand to put my control engineering
courses into practice. The gains come from trials and measurements on the
robot, not from a pendulum model with pole placement: I skipped that step to go
straight to reinforcement learning.

```
inner loop, 200 Hz, clocked by the MPU-6050 INT pin
    wheel acceleration = Kp·error + Ki·∫error + Kd·gyro      [steps/s²]
    wheel speed       += acceleration · dt                   the integrator IS the command
outer loop, 40 Hz
    target angle = offset − (Kp_v·speed error + Ki_v·position error)
auto-trim, very slow
    shifts the offset so the outer correction tends to zero
```

For an inverted pendulum, it is the acceleration of the contact point that
rights the body. A wheel speed proportional to the angle always leaves a
positive real pole, whatever the gains: that was the first version, and it could
not work.

**The policy** (command `2`): `politique.h`, a 15 → 32 → 32 → 2 tanh network,
1,634 weights in flash. It decides at 100 Hz and replaces **both loops**: its
inputs already include the position error and the speed setpoint. The speed
integrator, the limits and the safety checks stay in place.

> **The auto-trim does not run in policy mode.** It lives in the outer loop,
> which is disabled while the policy drives. Arm the dual PID **one minute
> before** switching to the policy, so the offset matches the true upright.

The full integration guide and the sign check procedure are in
[`INTEGRATION.md`](INTEGRATION.md) (in French). To regenerate the header after
retraining: `simulation/11_export_c.py` then `simulation/12_valider_c.py`,
**never one without the other**.

---

## Real-time architecture

**IMU reading in a dedicated task**, on core 0. Woken by the MPU-6050 INT pin,
it reads 8 bytes from register 0x3D (AY, AZ, TEMP, GX) and publishes the sample
under a lock. When the I2C transaction lived in `loop()`, a stuck bus blocked
the control loop and the watchdog rebooted the board: two crashes in 106 s of
shaking. After: 252 s without a single one, and 0 I2C failures at rest versus 1
per second.

**Step generation.** A timer ticks at 25 kHz. Each wheel has its own 16.16
fixed-point accumulator; when it overflows, that wheel takes a step
(Bresenham's algorithm). Two independent speeds from a single timer, hence
steering.

**Safety checks**, fastest first:

| | |
|---|---|
| dead man's switch in the ISR | 1,250 ticks (50 ms) without a new command: stepping stops |
| IMU silent 40 ms | command cut |
| IMU silent 150 ms | disarm, `>>> SECURITE : bus I2C fige` |
| IMU silent 3 s | board reboot |
| tilt > 30° | cut |
| watchdog | reboot if `loop()` hangs |
| black box | the current stage is written to RTC RAM, which survives a reboot: after a crash, the boot message says **where** the loop stopped |

---

## Serial commands

115,200 baud. One letter, optionally followed by a value (`N400`).

**Driving**

| cmd | effect |
|---|---|
| `1` | controller = dual PID |
| `2` | controller = policy |
| `K` | arm both loops, angle and position |
| `G` | angle loop only (the robot drifts, that is expected) |
| `S` | **stop**, also cancels a pending arm |
| `Z` | zero: the current angle becomes upright |
| `N`*sps* | speed setpoint in steps/s |
| `#`*sps* | speed difference between wheels, positive = turn right, ±2,000 |
| `W`*s* | duration of an `N` move; when non-zero, the robot goes out and back |

**Testing and measuring**

| cmd | effect |
|---|---|
| `3` | policy sign check, **wheels in the air**, motors silent |
| `4` | shake: full-amplitude reversals at 5 Hz, without IMU or feedback |
| `5` | motor stall ramp, **wheels in the air** |
| `6` | 200 Hz recording for 6 s, dumped as CSV (`ENR`) without disarming |
| `7` | 0.5 to 20 Hz sweep over 5 s, injected on acceleration. The robot vibrates in place. Arm the dual PID first (`1` then `K`). |
| `9` | 0.2 to 3 Hz sweep over 5.5 s on the angle setpoint, 1° amplitude. The robot rocks over about 2 cm. |
| `0` | moves about 10 cm forward after 1 s at rest, recorded, then comes back on its own. Arm the dual PID first. |
| `8` | prints a message from an old push protocol, no effect |
| `C` | restart gyroscope calibration |
| `V`*rpm* | manual mode with ramp, wheels in the air |

**Tuning** (saved to flash)

| cmd | parameter |
|---|---|
| `P` `I` `D` | inner loop gains |
| `T` `Y` | outer loop speed and position gains |
| `O` | angle offset |
| `L` | time constant of the speed filter |
| `U` `E` | start boost, maximum lead of the target |
| `M` `R` `B` | max speed, max acceleration, manual mode acceleration |
| `A` `H` `J` `X` | toggles: auto-trim, auto re-arm, arm at boot, outer loop sign |
| `F` `Q` `?` | force save, erase flash, print all parameters |

---

## Telemetry

One line every 100 ms:

```
P:-90.06 tgt:-90.46 e:0.40 g:6.2 PID DUAL sps:-19 c:-0.62 off:-91.08 pos:-165 hz:199 f:0 Lus:1225 Ius:1170 Tp:0 Tr:0
```

| field | meaning | expected |
|---|---|---|
| `P` | measured angle, degrees | upright around −91° (IMU on its side) |
| `tgt` | target angle | |
| `e` | angle error | |
| `g` | filtered gyro rate, °/s | ≈ 0 at rest |
| `PID` / `AGENT` | active controller | |
| `DUAL` / `ANG` / `OFF` | both loops, angle only, disarmed | |
| `sps` | wheel speed command, steps/s | saturated = fall imminent |
| `c` | outer loop correction, degrees | > 1° for long = wrong offset |
| `off` | angle offset | stable at rest |
| `pos` | odometry, in commanded steps | |
| `hz` | loop rate | 200 |
| `f` | cumulative I2C failures | 0 |
| `Lus` | worst loop iteration, µs | < 3,000 |
| `Ius` | worst I2C transaction, µs | < 1,500 |
| `Pus` | worst policy inference, µs (policy mode) | |
| `Tp` `Tr` | timer reprogrammings and restarts | 0 since the fixed-rate timer |
| `BLOC` | shown only if the I2C bus is stuck | |

---

## Tools, in `tools/`

A single program owns the serial port: **`serial_monitor.ps1`**. It finds the
board, writes everything to `logs/robot.log` with millisecond timestamps, sends
whatever is dropped into `tools/cmd.txt`, and releases the port by itself during
flashing. Creating `tools/PAUSE` releases the port, `tools/STOP` stops the
monitor. It does not always survive flashing: check it is running before
drawing conclusions from a silent log.

**`pupitre.py`** (control panel): 2D joystick for speed and steering, scale from
×0.5 to ×3, zeroing, controller selection, STOP (Escape key). It never touches
the port: it reads the log and writes to `cmd.txt`. Its buttons light up from
the telemetry, not from the last click, so a lost command is visible.

```
powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
python tools\pupitre.py
```
