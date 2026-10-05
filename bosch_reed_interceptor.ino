/*
 * ============================================================
 *  Bosch eBike 25 km/h limiter interceptor
 *  Rear-wheel reed switch, in-line (ONLY the reed is tapped)
 * ============================================================
 *
 * WHAT IT DOES
 *   Sits between the rear reed switch and the Bosch System
 *   Control Unit (SCU). It reads the real wheel pulses, works
 *   out your real speed, and emits a MODIFIED pulse train to
 *   the SCU. Pick a MODE below to decide what the SCU "sees".
 *
 * IMPORTANT LIMITATION (read this)
 *   With ONLY the reed switch tapped, the motor AND the display
 *   both read the SAME number (the SCU computes one speed from
 *   this one sensor and feeds it to both). So you CANNOT show a
 *   true >25 km/h on the display while keeping the motor in its
 *   <25 support band. This board keeps MOTOR SUPPORT alive (the
 *   real goal). Display behavior depends on MODE:
 *     - MODE_CAP  : display PINS at ~CAP_KMH (does NOT show >25)
 *     - MODE_SOFT : display CLIMBS and DOES show >25, but it is
 *                   scaled DOWN (not true) and support tapers
 *     - MODE_LINEAR: passthrough to RAMP_KNEE, then display = "1"+real/10
 *                   (0.1*real + 10; e.g. 56 -> 15.6). The reported speed
 *                   is SLEW-LIMITED (SLEW_KMH_PER_S) so the jump at the 25
 *                   knee becomes a ~1s fake brake (up-crossing) or fake
 *                   accel (down-crossing), not a hard step. Stays < 25,
 *                   so the motor never sees >25 and support never cuts.
 *   If you need the TRUE speed on the display AND full support,
 *   you must ALSO intercept the display<->SCU bus or reflash the
 *   SCU. That is a second tap, not "only the reed".
 *
 * WIRING (2-wire reed, the common case)
 *   - Cut the 2 wires from the rear reed to the SCU.
 *   - REED side:  reed terminal A -> REED_PIN,  reed terminal B -> GND
 *                  (board provides the pull-up. If your signal is
 *                   inverted, flip REED_ACTIVE_LOW.)
 *   - SCU side:   drive SCU_PIN. The SCU line is usually pulled
 *                  HIGH by the SCU; if it floats, add 10k from the
 *                  SCU_PIN line to 5V.
 *   - Power the Arduino from 5V (USB, or the sensor 5V if present).
 *   - If your "reed" is a 3-wire HALL (needs 5V+GND+signal), power
 *     it from 5V/GND and tap only the signal to REED_PIN.
 *   - Keep it in a small dry box; these live near the rear wheel.
 *
 * CALIBRATION (do once, with MODE = MODE_PASS)
 *   1. Set MODE = MODE_PASS, USE_SERIAL = 1, flash.
 *   2. Ride steady ~15-20 km/h, watch the serial "real=" value.
 *   3. Compare to phone GPS. Adjust WHEEL_CIRC_MM so they match:
 *          WHEEL_CIRC_MM_new = WHEEL_CIRC_MM_old * (gps_kmh / real_kmh)
 *   4. Repeat until "real" matches GPS. Then switch to CAP or SOFT.
 *   NOTE: your bike's own display speed is set by the SCU's own
 *     wheel-size config (dealer-set); WHEEL_CIRC_MM here only makes
 *     THIS board's internal estimate correct so the cap hits the
 *     right real speed.
 * ============================================================
 */

// ---------------- CONFIG ----------------
#define REED_PIN         2     // input  : real rear reed switch
#define SCU_PIN          3     // output : fake pulse to the SCU

// 1 if the REED line reads LOW when a magnet passes; 0 if it reads HIGH.
// (Most 2-wire reed-to-GND setups are LOW-active. Test & flip if wrong.)
#define REED_ACTIVE_LOW  1

#define WHEEL_CIRC_MM    2100.0f  // wheel+tire circumference in mm (CALIBRATE to GPS)
#define IN_PULSES_PER_REV   1     // real magnets on the wheel (usually 1)
#define OUT_PULSES_PER_REV  1     // fake pulses we emit per rev (keep 1)

// ---------------- POLICY ----------------
#define MODE_PASS  0   // stock passthrough (calibration / test)
#define MODE_CAP   1   // freeze reported speed at CAP_KMH -> max support, display pinned
#define MODE_SOFT  2   // passthrough to SOFT_KNEE, then scale down (display climbs, shows >25 but scaled)
#define MODE_LINEAR 3  // passthrough to RAMP_KNEE, then reported = RAMP_SLOPE*real + RAMP_INTERCEPT
#define MODE       MODE_LINEAR    // <--- PICK ONE

#define CAP_KMH     24.5f  // MODE_CAP: reported speed never exceeds this
#define SOFT_KNEE   25.0f  // MODE_SOFT: passthrough up to this
#define SOFT_RATIO  0.35f  // MODE_SOFT: fraction of excess speed reported above the knee

// MODE_LINEAR: above the knee, display = "1" + real/10  (== 0.1*real + 10)
//   e.g. 56 km/h -> 15.6. Reported stays < 25 for real < 150,
//   so the motor never sees >25 and support never cuts off.
#define RAMP_KNEE      25.0f  // passthrough (reported == real) at/below this
#define RAMP_SLOPE     0.1f   // above the knee: reported = SLOPE*real + INTERCEPT
#define RAMP_INTERCEPT 10.0f

#define OUT_PULSE_WIDTH_US 3000  // base output pulse width (auto-clamped to period/3)

#define USE_SERIAL   1
#define SERIAL_BAUD  115200

// ---------------- INTERNALS (tweak rarely) ----------------
#define SPEED_ALPHA    0.5f     // IIR on instantaneous period speed (per pulse)
#define STOP_TIMEOUT   1500UL   // no wheel pulse for this long -> ease speed to 0 (us)
#define STOP_DECAY     30.0f    // km/h per second the estimate decays once stopped
#define DEBOUNCE_US    3000UL
#define MIN_PULSE_US   20000UL  // ignore input edges closer than this
#define MAX_OUT_HZ     40.0     // safety clamp on emitted rate (no glitch bursts)
#define MAX_DT_US      10000UL  // clamp loop dt so a stall can't fire a pulse burst

// Slew-rate limit on the REPORTED speed (km/h per second). Turns the
// 25-knee jump into a ~1s fake brake/accel instead of a step. Normal
// riding (speed changes <~12 km/h/s) tracks real speed with no lag.
#define SLEW_KMH_PER_S 12.0f

#define OUT_OPEN_DRAIN 1        // 1 = open-drain (true reed mimic), 0 = push-pull

// ---------------- STATE ----------------
// input speed estimator: instantaneous period between wheel pulses + light IIR
float  smoothKmh = 0.0f;         // smoothed REAL speed (km/h)
bool   havePulse = false;        // seen a valid pulse (period available)
bool   countedThisPass = false;
bool   activePrev = false;
unsigned long lastChange = 0;
unsigned long lastPulseTime = 0; // micros() of last counted pulse

float  reportedKmh = 0.0f;   // speed we actually emit (slew-limited)
double outPhase = 0.0;
unsigned long lastLoop = 0;
bool   scuLow = false;
unsigned long pulseLowUntil = 0;

unsigned long lastPrint = 0;

// ---------------- OUTPUT DRIVERS ----------------
inline void scu_low() {
  #if OUT_OPEN_DRAIN
    pinMode(SCU_PIN, OUTPUT); digitalWrite(SCU_PIN, LOW);
  #else
    digitalWrite(SCU_PIN, LOW);
  #endif
}
inline void scu_high() {
  #if OUT_OPEN_DRAIN
    pinMode(SCU_PIN, INPUT_PULLUP);   // release: line returns to its pull-up (HIGH)
  #else
    digitalWrite(SCU_PIN, HIGH);
  #endif
}

// ---------------- MATH ----------------
inline float kmhFromHz(float hz) {
  float revsPerSec = hz / (float)IN_PULSES_PER_REV;
  return revsPerSec * (WHEEL_CIRC_MM / 1000.0f) * 3.6f;
}
inline float hzFromKmh(float kmh) {
  float revsPerSec = kmh / 3.6f / (WHEEL_CIRC_MM / 1000.0f);
  return revsPerSec * (float)OUT_PULSES_PER_REV;
}
float policyKmh(float realKmh) {
  switch (MODE) {
    case MODE_CAP:
      return realKmh < CAP_KMH ? realKmh : CAP_KMH;
    case MODE_SOFT:
      if (realKmh <= SOFT_KNEE) return realKmh;
      return SOFT_KNEE + (realKmh - SOFT_KNEE) * SOFT_RATIO;
    case MODE_LINEAR:
      if (realKmh <= RAMP_KNEE) return realKmh;
      return RAMP_SLOPE * realKmh + RAMP_INTERCEPT;
    case MODE_PASS:
    default:
      return realKmh;
  }
}

// ---------------- INPUT (real reed) ----------------
// Instantaneous speed from the period between wheel pulses (one per rev),
// light IIR for stability, ease-to-zero when the wheel stops.
void handleInput(unsigned long now, unsigned long dt) {
  bool lvl = (digitalRead(REED_PIN) == HIGH);
  bool active = REED_ACTIVE_LOW ? !lvl : lvl;   // true when magnet present

  if (active != activePrev) { activePrev = active; lastChange = now; }

  // debounce + count exactly once per magnet pass
  if (activePrev && (now - lastChange) >= DEBOUNCE_US) {
    if (!countedThisPass && (now - lastPulseTime) >= MIN_PULSE_US) {
      if (havePulse) {
        unsigned long period = now - lastPulseTime;   // us per pulse
        float instKmh = kmhFromHz(1000000.0f / (float)period);
        smoothKmh += SPEED_ALPHA * (instKmh - smoothKmh);
      }
      havePulse = true;
      lastPulseTime = now;
      countedThisPass = true;
    }
  } else if (!activePrev) {
    countedThisPass = false;
  }

  // no wheel motion for a while -> ease the estimate back to zero (parked = 0)
  if (havePulse && (now - lastPulseTime) > STOP_TIMEOUT) {
    smoothKmh -= STOP_DECAY * ((float)dt / 1000000.0f);
    if (smoothKmh < 0.0f) smoothKmh = 0.0f;
  }
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(SCU_PIN, OUTPUT);
  scu_high();                 // release the SCU line (HIGH via pull-up)
  lastLoop = micros();
  if (USE_SERIAL) {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println(F("Bosch reed interceptor ready. Calibrate WHEEL_CIRC_MM in MODE_PASS."));
  }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = micros();
  unsigned long dt = now - lastLoop; lastLoop = now;
  if (dt > MAX_DT_US) dt = MAX_DT_US;   // no burst if the loop ever stalls

  handleInput(now, dt);
  float realKmh   = smoothKmh;
  float targetKmh = policyKmh(realKmh);

  // slew-rate limit the reported speed toward the policy target, so the
  // 25-knee transition is a smooth fake brake (up) / accel (down), not a step
  float maxStep = SLEW_KMH_PER_S * ((float)dt / 1000000.0f);
  if (targetKmh > reportedKmh) reportedKmh += min(maxStep, targetKmh - reportedKmh);
  else                          reportedKmh -= min(maxStep, reportedKmh - targetKmh);

  float outHz = hzFromKmh(reportedKmh);
  if (outHz > MAX_OUT_HZ) outHz = MAX_OUT_HZ;

  // phase accumulator -> drift-free, glitch-free output at any rate
  if (outHz > 0.0f) {
    outPhase += (double)outHz * (double)dt / 1000000.0;
    while (outPhase >= 1.0) {
      unsigned long period_us = (unsigned long)(1000000.0 / (double)outHz);
      unsigned long width     = OUT_PULSE_WIDTH_US;
      if (width > period_us / 3) width = period_us / 3;
      scu_low();
      pulseLowUntil = now + width;
      scuLow = true;
      outPhase -= 1.0;
    }
  } else {
    outPhase = 0.0;
    if (scuLow) { scu_high(); scuLow = false; }
  }

  // enforce the pulse low-time without blocking
  if (scuLow && now >= pulseLowUntil) { scu_high(); scuLow = false; }

  if (USE_SERIAL && (now - lastPrint >= 500000)) {
    lastPrint = now;
    Serial.print(F("real="));     Serial.print(realKmh, 1);
    Serial.print(F(" reported="));Serial.print(reportedKmh, 1);
    Serial.println(F(" km/h"));
  }
}
