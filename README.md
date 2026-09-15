# Robot pendule inversé

*Inverted pendulum robot. The name stays in French: this project was made in France.*

**English** · [Français](README.fr.md)

**A two-wheel robot that balances on its own: first with a hand-tuned dual PID,
then with a neural network trained in simulation and running on the ESP32.**

<table>
<tr>
<td><img src="docs/media/apercu.gif" width="270" alt="The real robot pushed by hand, simulation in the corner"></td>
<td><img src="docs/media/mosaique.gif" width="270" alt="The 16 training checkpoints in simulation"></td>
</tr>
<tr>
<td align="center">real robot, same policy as in simulation</td>
<td align="center">16 training checkpoints, 9 M steps</td>
</tr>
</table>

Full video (45 s): see the [latest release](../../releases/latest).

> The code, its comments and the detailed documentation in the subfolders are
> in French. This page is the English entry point.

---

## Two ways to control the robot

The same firmware contains both, and you can switch between them live
(serial command `1` or `2`, or a button on the control panel).

| | **dual PID** (`1`) | **learned policy** (`2`) |
|---|---|---|
| origin | tuned by hand, to put my control engineering courses into practice | trained with PPO in MuJoCo |
| structure | 200 Hz angle loop → wheel acceleration, 40 Hz position loop that shifts the target angle, auto-trim of the upright angle | a 15 → 32 → 32 → 2 network, 1,634 weights, replacing both loops and deciding at 100 Hz |
| shared | IMU reading, speed integration, step generation, safety checks | same |

**In practice:** arm the dual PID for one minute first. Its auto-trim finds the
true upright angle, which the policy then reuses. The policy has no auto-trim.

The dual PID is also the **reference**: in simulation (`simulation/firmware.py`,
`08_agent.py --duel`) as on the robot, it tells whether a bad result comes from
the policy or from the hardware. On 9 September, run before the policy, it
balanced twelve times worse than earlier that evening: the problem was the
robot (discharged batteries: once recharged, everything worked again), not the
retraining.

---

## What's inside

```
hardware/     mechanics: SolidWorks CAD, STL, bill of materials, wiring, actual assembly
firmware/     ESP32 code: dual PID, learned policy, safety checks, serial commands
tools/        serial monitor (timestamped log) and control panel with joystick
simulation/   MuJoCo, Gymnasium environment, PPO, C export, videos
docs/         tuning notebook, project log (pitfalls, decisions, results)
donnees/      raw serial logs from the robot, source of every measured number
```

| | |
|---|---|
| [`hardware/README.md`](hardware/README.md) | parts, assembly, **differences from the CAD** (IMU moved) |
| [`hardware/wiring.md`](hardware/wiring.md) | ESP32, A4988, MPU-6050 pinout, power supply |
| [`firmware/README.md`](firmware/README.md) | real-time architecture, commands, telemetry, flashing |
| [`simulation/README.md`](simulation/README.md) | the model, training, export to the ESP32 |
| [`simulation/modeles/balancier.xml`](simulation/modeles/balancier.xml) | **the robot model**, in MJCF (MuJoCo's XML format) |
| [`docs/journal/PIEGES.md`](docs/journal/PIEGES.md) | bugs met along the way, what they cost, the rule learned |
| [`docs/journal/RESULTATS.md`](docs/journal/RESULTATS.md) | every number, with the command that produces it |

---

## Results

**The simulation matches the real robot.** The same policy, evaluated in
simulation and on the hardware:

| | simulation | real robot |
|---|---|---|
| tilt error, standard deviation | 1.743° | 1.572° |
| motor command, standard deviation | 873 steps/s | 878 steps/s |

1% difference on the command, about 10% on the tilt. The simulation was never
fitted to this observation: only the motor resonance and the sensor noise were
calibrated, each measured separately.

**Retrained on this faithful model**, the new policy, in simulation:

| | old policy | **new policy** | dual PID |
|---|---|---|---|
| tilt error, standard deviation | 1.743° | **0.106°** | 0.185° |
| command, standard deviation | 873 steps/s | **47 steps/s** | 86 steps/s |
| position drift | 55 mm | 12 mm | 13 mm |
| survival at maximum difficulty | 25% | **42%** | 33% |

On the robot, the new policy balanced for 76 s with a command standard
deviation of 536 steps/s, versus 893 for the old one. **A measured comparison
against the dual PID on the hardware is still to be done.**

The hand-tuned dual PID had first brought the tilt error down from 0.293° to
0.064° (see the [notebook](docs/carnet-du-balancier.html), in French). Its purpose
was to put my control engineering courses into practice, not to build the best
possible controller.

**I deliberately skipped the full control engineering approach**: modelling the
pendulum, deriving its transfer function, finding its poles and designing a
controller properly. The gains were found by trial and measurement on the
robot, and I then moved straight on to reinforcement learning, which was the
real goal of the project.

---

## Getting started

**Simulation** (Python 3.12, from `simulation/`):

```
pip install -r requirements.txt
pip install torch --index-url https://download.pytorch.org/whl/cpu

python modele.py --etat                 every parameter and where it comes from
python 01_regarder.py                   the robot in the viewer
python 08_agent.py --agent agents/balancier_final.zip
python 08_agent.py --duel               policy against dual PID
python 07_ppo.py --actionneur couple --pas 9000000       retrain (~16 min)
python 11_export_c.py --agent agents/balancier_final.zip
python 12_valider_c.py                  the C code must match PyTorch
```

**Firmware**: open `firmware/robot_balancier/robot_balancier.ino` in the
Arduino IDE, board *ESP32 Dev Module*, esp32 core 3.3.

**Driving the robot** (Windows):

```
powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
python tools\pupitre.py
```

---

## Timeline

| | |
|---|---|
| **August 2026** | frame, electronics, dual PID tuned by hand through trial and measurement, without a full model. A quick calculation shows that commanding wheel speed cannot stabilise an inverted pendulum: switch to commanding acceleration. A bug in Espressif's I2C driver tracked down with `addr2line`. |
| **5 Sept** | MuJoCo model rebuilt from the SolidWorks assembly, firmware ported to simulation as a reference, first PPO policy. |
| **9 Sept** | policy exported to C and validated against PyTorch. Board crashes fixed by moving I2C to a dedicated task. Motor identified in closed loop with frequency sweeps and cross-spectra, without taking the robot apart: resonance at 71 Hz, not 113. Retraining on the faithful motor model. |
| **10 Sept** | steering (two independent wheel speeds from a single timer), joystick control panel, video. |

## Still open

- measure the dual PID and the policy on the hardware, under the same conditions;
- set the A4988 Vref, never measured;
- the period of the robot hanging as a pendulum, an independent check of the model ([`docs/MESURES.md`](docs/MESURES.md)).

---

**Matthieu Vinet**, robotics engineering student at Polytech Sorbonne ·
[matthieu-vinet.fr](https://matthieu-vinet.fr/)

Code written with Claude Code as a programming assistant. [MIT](LICENSE) license.
