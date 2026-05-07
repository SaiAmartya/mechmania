# Stepper Motor Wiring Guide

Wiring reference for the conveyor stepper subsystem on the MechMania
competition robot:

- **MCU:** Firebeetle 2 ESP32-E (receiver)
- **Driver:** A4988 stepstick
- **Motor:** NEMA 17 (bipolar, 4-wire)
- **Sensor:** Cherry SPDT lever microswitch (bottom-of-travel limit)

> ⚠️ **Power off the board every time you change wiring.** Hot-plugging a
> stepper motor into the A4988 will destroy the driver IC.

---

## 1. ESP32 → A4988 (logic side)

| A4988 pin   | ESP32 pin | Notes |
|-------------|-----------|-------|
| `STEP`      | GPIO 13   | One pulse per microstep. Code toggles this every 600 µs. |
| `DIR`       | GPIO 2    | HIGH/LOW selects rotation direction. |
| `VDD`       | 3V3       | Logic supply (3.3 V is fine; A4988 accepts 3 – 5.5 V). |
| `GND`       | GND       | Logic ground. **Must be common with motor-supply GND** (see §3). |
| `ENABLE`    | GND       | Tie LOW to keep the driver enabled. (Active-low.) |
| `SLEEP`     | jumper to `RESET` | `SLEEP` has an internal pull-up; jumpering to `RESET` (which doesn't) brings both HIGH and lets the chip run. Standard A4988 trick. |
| `RESET`     | jumper to `SLEEP` | Same jumper, both ends. |
| `MS1` `MS2` `MS3` | leave open | All floating = full-step mode. Pull `MS1` HIGH for half-step if you need smoother motion. |

That's it for the MCU side — only **2 GPIOs** drive the motor; everything
else is power, ground, or static-level jumpers.

---

## 2. A4988 → NEMA 17 (motor side)

A NEMA 17 has **two coils** (4 wires total). The A4988 has 4 motor outputs:
`1A`, `1B`, `2A`, `2B`. Each coil connects to one **A/B pair**.

| A4988 pin | Coil |
|-----------|------|
| `1A`      | Coil 1 lead |
| `1B`      | Coil 1 lead |
| `2A`      | Coil 2 lead |
| `2B`      | Coil 2 lead |

**Identifying coil pairs on the motor** (do this with a multimeter before
wiring — color codes vary between manufacturers):

1. Measure resistance between every pair of motor wires.
2. The two wires with **low resistance** (~2 – 4 Ω) are the same coil.
3. Wires from different coils show **no continuity** (open circuit).

You'll end up with two pairs. Wire one pair to `1A`/`1B`, the other to
`2A`/`2B`. **Order within a pair only changes rotation direction** — if
the conveyor runs the wrong way, swap the two wires of *one* coil (or
flip `CONVEYOR_DIR_FORWARD` in firmware).

A common 17HS-series color code (for reference, but always verify):

| Wire color | Coil    |
|------------|---------|
| Black      | Coil A+ |
| Green      | Coil A− |
| Red        | Coil B+ |
| Blue       | Coil B− |

---

## 3. Motor power on the A4988

| A4988 pin | Connect to | Notes |
|-----------|------------|-------|
| `VMOT`    | + of motor supply (8 – 35 V; we run ~12 V) | Stepper coil power. |
| `GND` (motor side) | − of motor supply **and** ESP32 GND | Common ground is mandatory. |

**Required:** a **100 µF electrolytic capacitor** across `VMOT` ↔ `GND`,
mounted as close to those pins as possible. The A4988 datasheet calls
this out explicitly — without it, switching transients can spike past the
35 V absolute max and **kill the chip**. Mind polarity (longer leg = +).

### Current limit (Vref) — set this before running the motor

The A4988's small trim-pot sets the per-coil current limit. Procedure:

1. Power VMOT. Logic side can be unpowered for this step.
2. Touch a multimeter between the wiper of the trim-pot and GND.
3. Adjust until `Vref` matches your target current.

Target: start at the lower of your motor's rated current per phase or
~70 % of the A4988's safe ceiling (1 A without a heatsink, ~2 A with
active cooling).

Vref formula depends on the sense-resistor value the specific stepstick
clone uses:

- **R_S = 0.100 Ω** (common Pololu / generic): `I = Vref / 0.8`
- **R_S = 0.050 Ω** (some "rev 4" clones):     `I = Vref / 0.4`

Inspect the two tiny SMD resistors near the motor outputs — they're
labeled `R100` (= 0.10 Ω) or `R050` (= 0.05 Ω). When in doubt, **start
low** (Vref = 0.4 V), confirm the motor turns smoothly, then nudge up
until torque is adequate but the driver isn't visibly hot.

---

## 4. Cherry limit switch (bottom-of-travel cutoff)

The cherry microswitch is an SPDT lever switch with three terminals:
`COM`, `NO` (normally open), `NC` (normally closed). For this robot
**only two terminals are used**: `COM` and `NO`.

| Switch pin | Connect to | Notes |
|------------|------------|-------|
| `COM`      | ESP32 `GND` | |
| `NO`       | ESP32 `GPIO 27` | Internal pull-up enabled in firmware — no external resistor needed. |
| `NC`       | leave unconnected | |

**Behavior:**

- Hook **off** the switch (lever released): NO is open → GPIO reads
  **HIGH** (pulled up internally). Conveyor can move both directions.
- Hook **pressing** the switch (lever depressed): NO closes to GND →
  GPIO reads **LOW**. Firmware blocks "down" (btn2) but still allows
  "up" (btn1) so the hook can drive off the switch.

**Mounting:** position the switch at the bottom of the hook's travel
such that the lever is depressed **slightly before** the hook would hit
the frame — the few microseconds between detection and the next loop
iteration don't matter, but mechanical overshoot does.

**Wire the switch with short, well-secured leads.** Long unshielded
leads next to the stepper power wires can pick up enough noise to
falsely trigger; if you see phantom presses, twist the COM/NO pair or
add a small (~100 nF) cap from GPIO 27 to GND at the ESP32 end.

---

## 5. Pin summary (receiver ESP32)

| Function          | GPIO | Notes |
|-------------------|------|-------|
| Conveyor `STEP`   | 13   | → A4988 `STEP` |
| Conveyor `DIR`    | 2    | → A4988 `DIR` (also a boot-strap pin; keep the line clean) |
| Limit switch      | 27   | → cherry NO; INPUT_PULLUP, LOW = pressed |
| `3V3`             | —    | → A4988 `VDD` |
| `GND`             | —    | → A4988 logic GND, A4988 motor GND, motor PSU −, switch COM |

---

## 6. Pre-flight checklist

- [ ] 100 µF cap installed across `VMOT` / `GND`
- [ ] `SLEEP` ↔ `RESET` jumpered
- [ ] `ENABLE` tied to GND
- [ ] Coil pairs verified with a multimeter (low-resistance pair on `1A`/`1B`, the other on `2A`/`2B`)
- [ ] Motor PSU GND and ESP32 GND are common
- [ ] Vref set conservatively (start at 0.4 V, tune up as needed)
- [ ] Cherry switch lever depresses *before* hook reaches the mechanical limit
- [ ] Power applied **VMOT first, then logic** (and reverse on shutdown)
