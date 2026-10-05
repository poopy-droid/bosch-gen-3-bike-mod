# Bosch eBike 25 km/h Limiter — Reed Interceptor

**Firmware:** v1.3
**Platform:** Arduino
**Sensor:** Rear wheel reed switch

> ⚠️ **UNTESTED:** I was too lazy to test this on a bike. Check everything yourself before riding. If something goes to shit, that's your responsibility.

> ⚠️ **LEGAL:** Check your local laws before using this. If you get busted, that's not my fault. Modifying the speed limiter may affect the bike's legal status, insurance, warranty, and where it can be ridden.

---

## What it does

The Arduino sits inline between the rear wheel reed switch and the Bosch SCU, using the existing cable.

It reads wheel speed from the rear reed pulses and sends the modified signal back to the SCU.

| Mode                        | Description                                                                                                   |
| --------------------------- | ------------------------------------------------------------------------------------------------------------- |
| **PASS (passthrough mode)** | The Arduino passes the real wheel-speed signal through to the Bosch SCU without modifying the reported speed. |
| **SPEED MODE**              | Above 25 km/h, reported speed follows `0.1 × real + 10`.                                                      |

---

## Speed Mode

The safety reed selects the speed mode with a magnet:

**MAGNET AWAY** → Reed open → **PASS**

**MAGNET NEAR** → Reed closed → **SPEED MODE**

The safety reed can be disabled in the code. When disabled, the unit stays in SPEED MODE.

You can turn the safety reed **on or off** in the code using `true` or `false`.

The idea is that the magnet has to be deliberately positioned for SPEED MODE. If someone else uses the bike without the safety magnet positioned correctly, **PASS** is active. Wink wink.

---

## SPEED MODE

`reported = 0.1 × real + 10`

| Real | Reported |
| ---: | -------: |
| 25.0 |     25.0 |
| 25.1 |    ~12.5 |
|   30 |       13 |
|   40 |       14 |
|   50 |       15 |
|   55 |     15.5 |
|   56 |     15.6 |

---

## Slew Stepping

The reported speed moves toward the calculated value at a maximum rate of:

`SLEW_KMH_PER_S = 12.0`

Instead of an immediate change at 25 km/h, the output steps toward the target over time.

Theoretically, this makes the SCU less likely to detect misuse from a sudden speed-signal change.

---

## Wiring

The Arduino goes inline in the existing **rear reed → SCU** cable.

### Rear wheel reed switch

```text id="n0wjx3"
Reed signal → Arduino D2 (REED_PIN) 
Reed ground → GND 
```

### Arduino → SCU

```text id="v0rjck"
Arduino D3 (SCU_PIN) → existing signal wire to SCU 
Arduino GND → SCU GND 
```

### Safety reed switch

```text id="0t9gl2"
Reed signal → Arduino D4 (KILL_PIN) 
Reed ground → GND 
```

## Configuration

| Setting              | Default |
| -------------------- | ------: |
| `REED_PIN`           |       2 |
| `KILL_PIN`           |       4 |
| `SCU_PIN`            |       3 |
| `WHEEL_CIRC_MM`      |  2100.0 |
| `IN_PULSES_PER_REV`  |       1 |
| `OUT_PULSES_PER_REV` |       1 |
| `RAMP_KNEE`          |    25.0 |
| `RAMP_SLOPE`         |     0.1 |
| `RAMP_INTERCEPT`     |    10.0 |
| `SLEW_KMH_PER_S`     |    12.0 |
| `OUT_PULSE_WIDTH_US` |    3000 |

All settings are in `bosch_reed_interceptor.ino`.

---

## Rev History

**v1.0** — Initial interceptor.
**v1.2** — PASS + SPEED MODE, slew limiting.
**v1.3** — Safety reed Added
