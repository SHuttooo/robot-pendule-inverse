# Raw robot data

**English** · [Français](README.fr.md)

`logs_robot_2026-08-28_au_2026-09-12.zip` (3.7 MB compressed, 31 MB raw):
everything that went through the robot's serial port, timestamped to the
millisecond by `tools/serial_monitor.ps1`. It is the source of every number
measured on the hardware in this repository.

| file | period | content |
|---|---|---|
| `robot-avant.log` | up to 28 August | tuning of the dual PID (notebook) |
| `robot.log.old` | up to 10 Sept, 19:12 | I2C crash, motor identification, first policy run |
| `robot.log` | 10 to 12 Sept | steering, control panel, the video take of 10 Sept at 21:06 |

**Pitfall:** the log rotates at 20 MB. All of 9 September is therefore in
`robot.log.old`, and an analysis run on `robot.log` alone gives wrong numbers.
It happened.

To use it: unzip into `logs/` at the repository root (the folder is ignored by
git).

## Reading a telemetry line

```
10:53:31.710 P:-90.06 tgt:-90.46 e:0.40 g:6.2 PID DUAL sps:-19 c:-0.62 off:-91.08 pos:-165 hz:199 f:0 Lus:1225 Ius:1170 Tp:0 Tr:0
```

Every field is described in [`firmware/README.md`](../firmware/README.md#telemetry).
