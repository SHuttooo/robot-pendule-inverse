# Mechanics and hardware

**English** · [Français](README.fr.md)

<table>
<tr>
<td><img src="../docs/media/robot_cote.jpg" width="260" alt="The robot from the side"></td>
<td><img src="../docs/media/robot_face.jpg" width="260" alt="The robot from the front"></td>
<td><img src="../docs/media/robot_trois_quarts.jpg" width="260" alt="The robot, three-quarter view"></td>
</tr>
</table>

*Frames taken from the video of 10 September 2026.*

An inverted pendulum on two wheels. The blue bottom part holds both motors; the
three red levels above carry the breadboard, the ESP32 and, at the very top,
the holder for the four batteries.

Wiring is described in [`wiring.md`](wiring.md).

---

## Bill of materials

| component | part | qty | note |
|---|---|---|---|
| microcontroller | ESP32 WROOM-32D, development board | 1 | |
| IMU | MPU-6050 | 1 | I2C, address 0x68 |
| motor drivers | A4988 | 2 | 1/8 microstepping, i.e. 1,600 steps/rev (measured) |
| motors | NEMA 17 17HS3401S | 2 | |
| batteries | 18650 Li-ion 3.7 V LiitoKala | 4 | in series: 4S, 14.8 V nominal |
| converter | step-down to 5 V | 1 | powers the ESP32 through its VIN pin |
| capacitor | 100 µF | 1 | on VMOT |
| breadboard | | 1 | |
| wheels | Ø 65 mm, 26 mm wide | 2 | |
| printed parts | `piece1` to `piece4` | 4 | see below |
| threaded inserts | M2.5 | 2 | planned for the original IMU location, never used in the end |
| double-sided tape | | | holds the ESP32, breadboard, batteries and IMU |

---

## The parts

| part | bounding box | printed volume |
|---|---|---|
| `piece1` | 113 × 42 × 47 mm | 92.8 cm³ |
| `piece2` | 143 × 75 × 45 mm | 65.2 cm³ |
| `piece3` | 143.6 × 75.6 × 80 mm | 118.1 cm³ |
| `piece4` | 143.7 × 75.7 × 63 mm | 103.5 cm³ |

Assembled robot: 143.7 × 75.7 × 220 mm, 231.5 mm tall including wheels. Shelves
at +66, +141 and +199 mm above the wheel axle. All these dimensions are read
from the assembly by `simulation/modele.py`, not measured with calipers.

### Files

| folder | content |
|---|---|
| `cao/solidworks/` | `Assemblage1.SLDASM` and its 5 parts. **Version of 25 August 2026, the latest.** |
| `cao/stl/` | the 4 parts to print. `piece2` to `piece4` are the 25 August exports (formerly `v2`). |
| `cao/stl_assemblage/` | the 6 STL files exported in the assembly frame, loaded by the simulation |

### The 25 August revision

Comparing the meshes before and after: the outer shape of the parts does not
change. A single contour changes, 5 mm high, at the top of each part.

| part | before | after | per side |
|---|---|---|---|
| `piece2` (tenon) | 107.90 × 36.90 mm | 107.70 × 36.70 mm | −0.10 mm |
| `piece3` | 130.74 × 66.90 mm | 130.46 × 66.70 mm | −0.14 / −0.10 mm |
| `piece4` | 130.73 × 62.73 mm | 130.45 × 62.45 mm | −0.14 mm |

This is clearance added so the levels fit into each other.

The STL files in `cao/stl_assemblage/` were exported **before** this revision
(checked on the geometry: 0.01 mm from the old `piece2`, 0.11 mm from the new
one). A 0.1 mm wall change has no visible effect on the model's mass and
inertia. To align them anyway: re-export the assembly as STL, one file per part,
in the assembly frame.

---

## The actual robot differs from the CAD

### The IMU was moved away from the motors

The SolidWorks assembly shows the **planned** IMU location, not the one on the
robot:

| | part | location | mounting |
|---|---|---|---|
| **planned in the CAD** | `piece1`, the bottom part | the one holding both motors | 2 M2.5 threaded inserts |
| **on the robot** | `piece2`, second from the bottom | on the side of the central leg that supports it | double-sided tape |

Why: a stepper motor moves in small jerks, 1,600 per revolution. Mounted close
to it, the IMU picks up these vibrations on top of the robot's motion, and the
gyroscope reads them as rotations. The controller's derivative term then
amplifies them. At the start of tuning, the notebook recorded 3.48 °/s of
gyroscope noise.

The two **M2.5 inserts** planned to hold it on the motor part were therefore
never used. If `piece1` is printed again, their holes can be ignored.

**Orientation, to keep if it is moved again.** The firmware only reads `AY`,
`AZ` and `GX`: the sensor's X axis must stay parallel to the wheel axle. The
sensor is mounted on its side, so upright reads around −91.4° instead of 0°.
The `Z` command resets that zero while the robot is held upright.

### Mounting with double-sided tape

The **ESP32**, the **breadboard**, the **batteries** and the **IMU** are stuck
on with double-sided tape, no screws.

Two consequences:

- a taped part can slip after a fall. If the IMU moved, the balance point moves
  with it: redo the zero (`Z`), then let the dual PID run for a minute so the
  auto-trim finds upright again;
- for the IMU, **thick foam** double-sided tape also filters part of the motor
  vibrations.

---

## Things to watch

| | |
|---|---|
| **A4988 Vref** | never set. It sets the available torque. Procedure: `Vref = I × 8 × Rshunt`, see the [notebook](../docs/carnet-du-balancier.html) (in French). |
| **ENABLE** | not wired: the motors stay powered and heat up even when idle. |
| **Battery voltage** | not measured. One cell died without warning on 9 September 2026. A voltage divider to an analog input would fix this. |
| **USB and 5 V** | never power the ESP32 from USB and from the converter at the same time. Keep the grounds common, disconnect the 5 V wire. |
| **History** | a first ESP32 was destroyed by 12 V on its 3V3 pin. The drivers and the IMU were replaced, the converter was not. |
