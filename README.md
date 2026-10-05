# Bosch 25 km/h Limiter — Reed Interceptor

Arduino-based inline interceptor that sits between the rear-wheel reed switch and the
Bosch System Control Unit (SCU) to modify the speed pulses the SCU sees.
Read the [important limitation](#important-limitation) before flashing anything.

## ⚠️ Disclaimer (read this first)

- **This project is UNTESTED.** I have not flashed it on a bike or ridden with it —
  I was too lazy to bother, so treat everything here as unproven until you test it
  yourself. Verify wiring polarity, pulse behavior, and mode behavior in a safe
  setting first.
- **Do not use this on a road bike in a hurry.** Modifying the 25 km/h limiter on a
  Bosch-assist e-bike may violate local laws (classifying the bike differently,
  warranty voiding, insurance/liability issues, speed-limit regulations).
  **If it is against the law where you live, don't use it.** This is your decision
  and your risk, not mine.
- For calibration/test mode only: `MODE_PASS`.

## What it does

Sits inline between the reed and the SCU, reads real wheel pulses, computes real
speed, emits a modified pulse train to the SCU. **Motor support stays alive** —
that's the goal.

### Important limitation (single tap only)

With **only the reed switched**, the motor and the display both read the **same**
number (SCU computes one speed from this sensor and feeds both). So you **cannot**
show a true >25 km/h on the display while keeping the motor in its <25 support
band. Display behavior depends on mode:

| Mode | Display | Motor support |
|------|---------|---------------|
| `MODE_CAP` | Pinned at ~CAP_KMH (never shows >25) | Full |
| `MODE_SOFT` | Climb and shows >25, but **scaled down** (not true speed) | Tapers |
| `MODE_LINEAR` | `reported = 0.1×real + 10` above knee (56 → 15.6); stays <25 so support never cuts | Full |
| `MODE_PASS` | True passthrough (calibration) | True |

> If you need TRUE speed on the display AND full support, you must **also**
> intercept the display↔SCU bus or reflash the SCU. That's a **second tap**,
> not "only the reed".

## Wiring (2-wire reed — the common case)

1. Cut the 2 wires from the rear reed to the SCU.
2. **Reed side:** reed terminal A → `REED_PIN` (Arduino pin 2), reed terminal B → GND.
   (Board provides the pull-up. If your signal is inverted, flip `REED_ACTIVE_LOW`.)
3. **SCU side:** drive `SCU_PIN` (Arduino pin 3). The SCU line is usually pulled
   HIGH by the SCU; if it floats, add 10 kΩ from `SCU_PIN` line to 5V.
4. Power the Arduino from 5V (USB, or the sensor 5V if present).
5. 3-wire HALL sensor (needs 5V+GND+signal)? Power it from 5V/GND and tap only the
   signal to `REED_PIN`.
6. Keep it in a small **dry box** — this lives near the rear wheel.

## Configuration

All settings in `bosch_reed_interceptor.ino`:

```c
#define REED_PIN         2     // input  : real rear reed switch
#define SCU_PIN          3     // output : fake pulse to the SCU
#define REED_ACTIVE_LOW  1     // 1 = reed line goes LOW when magnet passes
                              //    0 = reed line goes HIGH (test & flip if wrong)
#define WHEEL_CIRC_MM    2100.0f  // wheel+tire circumference in mm (CALIBRATE to GPS)
#define IN_PULSES_PER_REV   1     // real magnets on the wheel (usually 1)
#define OUT_PULSES_PER_REV  1     // fake pulses we emit per rev (keep 1)
```

Modes:

```c
#define MODE_PASS  0   // stock passthrough (calibration / test)
#define MODE_CAP   1   // freeze reported speed at CAP_KMH -> max support, display pinned
#define MODE_SOFT  2   // passthrough to SOFT_KNEE, then scale down
#define MODE_LINEAR 3  // passthrough to RAMP_KNEE, then reported = RAMP_SLOPE*real + RAMP_INTERCEPT
#define MODE       MODE_LINEAR    // <--- PICK ONE
```

Tuning constants:

```c
#define CAP_KMH     24.5f  // MODE_CAP: reported speed never exceeds this
#define SOFT_KNEE   25.0f  // MODE_SOFT: passthrough up to this
#define SOFT_RATIO  0.35f  // MODE_SOFT: fraction of excess speed reported above the knee
#define RAMP_KNEE      25.0f  // passthrough (reported == real) at/below this
#define RAMP_SLOPE     0.1f   // above the knee: reported = SLOPE*real + INTERCEPT
#define RAMP_INTERCEPT 10.0f
#define OUT_PULSE_WIDTH_US 3000  // base output pulse width (auto-clamped to period/3)
#define SLEW_KMH_PER_S 12.0f   // slew-rate limit on the REPORTED speed (km/h/s)
#define USE_SERIAL   1
#define SERIAL_BAUD  115200
```

## Calibration (do once, with `MODE = MODE_PASS`)

1. Set `MODE = MODE_PASS`, `USE_SERIAL = 1`, flash.
2. Ride steady ~15–20 km/h, watch the serial "real=" value.
3. Compare to phone GPS. Adjust `WHEEL_CIRC_MM` so they match:

   ```
   WHEEL_CIRC_MM_new = WHEEL_CIRC_MM_old * (gps_kmh / real_kmh)
   ```

4. Repeat until "real" matches GPS. Then switch to your chosen mode (CAP, SOFT, or LINEAR).

> NOTE: your bike's own display speed is set by the SCU's own wheel-size
> config (dealer-set). `WHEEL_CIRC_MM` here only makes **this board's**
> internal estimate correct so the cap hits the right real speed.

## How it works (brief)

- **Input:** period between wheel pulses → instantaneous speed → light IIR smoothing
  (`SPEED_ALPHA`) → ease-to-zero when parked (`STOP_TIMEOUT`).
- **Policy:** `policyKmh(realKmh)` picks the reported speed based on `MODE`.
- **Slew-rate limit:** `SLEW_KMH_PER_S` turns the 25 km/h knee into a ~1 s fake
  brake (up-crossing) / fake accel (down-crossing), not a hard step.
- **Output:** phase accumulator → drift-free, glitch-free pulse train at any rate.
  `OUT_PULSE_WIDTH_US` clamped to `period/3`; `MAX_OUT_HZ` safety cap (40 Hz).
- **Serial monitor** (115200): prints `real=` vs `reported=` every 0.5 s.

## Status

- [ ] Wiring verified against a real bike
- [ ] `MODE_PASS` calibrated against GPS
- [ ] `MODE_CAP` / `MODE_SOFT` / `MODE_LINEAR` behavior verified under load
- [ ] Legal check done for your region

**If any of those are unchecked — don't put it on a bike you care about.**
