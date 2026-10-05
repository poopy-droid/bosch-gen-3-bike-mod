/*
 * ============================================================
 *  Bosch eBike 25 km/h limiter interceptor
 *  Wired inline in the existing rear reed -> SCU cable. ONLY the reed is tapped.
 *  Firmware v1.3
 * ============================================================
 *
 * WHAT IT DOES
 *   Sits between the rear reed switch and the Bosch System
 *   Control Unit (SCU). It reads the real wheel pulses, works
 *   out your real speed, and emits a MODIFIED pulse train to
 *   the SCU. No GPS, no extra sensors — speed is pure reed
 *   pulse timing (same math the SCU uses).
 *
 *   THE SAFETY REED SELECTS THE MODE WITH A MAGNET:
 *     MAGNET AWAY  (reed OPEN)  -> PASS        (true speed, stock)
 *     MAGNET NEAR  (reed CLOSED) -> SPEED MODE (fast, display scaled)
 *   The idea: SPEED MODE needs the magnet deliberately positioned.
 *   If someone else rides without it, PASS (stock) is active.
 *   Wink wink.
 *   The flag USE_MODE_REED disables the safety reed entirely:
 *     USE_MODE_REED = false -> safety reed ignored -> ALWAYS SPEED MODE.
 *
 * IMPORTANT LIMITATION
 *   With only the reed tapped, motor AND display read the SAME
 *   number (SCU computes one speed from this sensor and feeds
 *   both). You CANNOT show a true >25 km/h on the display while
 *   keeping motor in its <25 support band. This board keeps
 *   MOTOR SUPPORT alive (the real goal). Display behavior by mode:
 *     - PASS: display = real speed (stock)
 *     - SPEED MODE: display climbs and shows >25, but SCALED DOWN
 *               (0.1*real + 10; e.g. 56 -> 15.6). Reported speed
 *               is SLEW-limited (SLEW_KMH_PER_S), so the jump at
 *               the 25-km/h knee becomes a ~1s fake brake / fake
 *               accel instead of a hard step. It stays < 25, so
 *               the motor never sees >25 and support never cuts.
 *   True speed on display AND full support needs a 2nd tap
 *   (display<->SCU bus) or an SCU reflash.
 *
 * WIRING (two reed switches, 2-wire each)
 *   MAIN reed (rear wheel):
 *     - Cut the 2 wires from the rear reed to the SCU.
 *     - terminal A -> REED_PIN, terminal B -> GND
 *       (board pull-up; flip REED_ACTIVE_LOW if inverted)
 *   SAFETY reed (mount wherever handy: frame, handlebar, bag):
 *     - terminal A -> KILL_PIN, terminal B -> GND
 *       (board pull-up; flip KILL_ACTIVE_LOW if needed)
 *     - magnet present = SPEED MODE (fast); magnet away = PASS
 *   SCU side:
 *     - drive SCU_PIN (SCU line is usually pulled HIGH by the
 *       SCU; add 10k to 5V if it floats)
 *     - Arduino GND -> SCU GND
 *   Power the Arduino from 5V (USB or the sensor 5V). 3-wire
 *   HALL? Power from 5V/GND, tap only signal to REED_PIN.
 *   Keep it in a small dry box.
 *
 * THE SLEW (why speed doesn't snap)
 *   SPEED MODE jumps the *reported* speed at the knee: 25.1 real ->
 *   ~12.5 reported. Without slew the output frequency would step
 *   12.5Hz -> ~1Hz instantly (hard jerk on the motor). SLEW_KMH_
 *   PER_S caps how fast reported speed can move, so crossing 25
 *   feels like a ~1s fake brake (up) / fake accel (down).
 *   Normal riding changes speed slower than that = no lag.
 *   Theoretical: this makes the SCU less likely to detect misuse
 *   from a sudden speed-signal change.
 *
 * CALIBRATION (once, with the safety reed OPEN = PASS)
 *   1. Flash with USE_SERIAL = true, safety magnet AWAY.
 *   2. Ride steady 15-20 km/h, watch the serial "real=" value.
 *   3. Compare to a reference (phone GPS, bike display, or
 *      tape-measured wheel). Adjust:
 *          WHEEL_CIRC_MM_new = WHEEL_CIRC_MM_old * (ref_kmh / real_kmh)
 *   4. Repeat until "real" matches. Then: magnet present = SPEED
 *      MODE (fast), magnet away = stock (PASS).
 *   NOTE: bike display speed is set by the SCU's own (dealer-set)
 *   wheel size; WHEEL_CIRC_MM only fixes THIS board's estimate.
 * ============================================================
 */

#define FIRMWARE_REV "v1.3"


// ---------------- CONFIG ----------------
#define REED_PIN      2     // input  : real rear reed switch
#define KILL_PIN      4     // input  : safety reed (magnet present = SPEED MODE)
#define SCU_PIN       3     // output : pulse to the SCU

// true = line goes LOW when magnet present; false = line goes HIGH.
// (Most 2-wire reed-to-GND setups are LOW-active. Test & flip if wrong.)
#define REED_ACTIVE_LOW  true
#define KILL_ACTIVE_LOW  true

// wheel+tire circumference in mm (CALIBRATE ONCE, see header)
#define WHEEL_CIRC_MM     2100.0f
// magnet count on the wheel (usually 1) and fake pulses emitted per rev (keep 1)
#define IN_PULSES_PER_REV   1
#define OUT_PULSES_PER_REV  1

// ---------------- POLICY ----------------
// The safety reed can be turned ON or OFF in the code using true or false:
//   USE_MODE_REED = true  : safety reed ON  -> magnet near -> SPEED MODE, magnet away -> PASS
//   USE_MODE_REED = false : safety reed OFF -> ALWAYS SPEED MODE (no magnet needed)
#define USE_MODE_REED true

// SPEED MODE, above the knee: display = 0.1*real + 10  (e.g. 56 -> 15.6)
// Reported stays < 25 for real < 150, so support never cuts.
#define RAMP_KNEE      25.0f  // reported == real at/below this
#define RAMP_SLOPE     0.1f   // reported = SLOPE*real + INTERCEPT above it
#define RAMP_INTERCEPT 10.0f

// base output pulse width, auto-clamped to period/3
#define OUT_PULSE_WIDTH_US 3000

// ---------------- SLEW ----------------
// max rate of change of the *reported* speed (km/h per second);
// turns the 25-km/h knee jump into a ~1s fake brake/accel, not a step.
// Theoretical: makes the SCU less likely to notice a sudden speed-signal change.
#define SLEW_KMH_PER_S 12.0f

#define USE_SERIAL   true
#define SERIAL_BAUD  115200

// ---------------- INTERNALS (tweak rarely) ----------------
// IIR coefficient on the instantaneous period speed (per pulse)
#define SPEED_ALPHA      0.5f
// no wheel pulse for this long (us) -> estimate eases back to 0
#define STOP_TIMEOUT     1500UL
// km/h per second the estimate decays once stopped
#define STOP_DECAY       30.0f
// input edge debounce (us)
#define DEBOUNCE_US      3000UL
// mode-reed level debounce (us)
#define KILL_DEBOUNCE_US 50000UL
// ignore input edges closer than this (us)
#define MIN_PULSE_US     20000UL
// safety clamp on emitted rate (no glitch bursts)
#define MAX_OUT_HZ       40.0
// clamp loop dt so a stall can't fire a pulse burst
#define MAX_DT_US        10000UL

// true = open-drain (true reed mimic), false = push-pull
#define OUT_OPEN_DRAIN   true

// ---------------- STATE ----------------
// speed estimator: instantaneous period between wheel pulses + light IIR
float  smoothKmh = 0.0f;         // smoothed REAL speed (km/h)
bool   havePulse = false;        // seen a valid pulse (period available)
bool   countedThisPass = false;
bool   activePrev = false;
unsigned long lastChange = 0;
unsigned long lastPulseTime = 0; // micros() of last counted pulse

// output state
float  reportedKmh = 0.0f;   // speed we actually emit (slew-limited)
double outPhase    = 0.0;
unsigned long lastLoop = 0;
bool   scuLow      = false;
unsigned long pulseLowUntil = 0;

// safety reed state (debounced): true = magnet present => SPEED MODE
bool   killRawPrev = false;
unsigned long killChange = 0;
bool   killActive  = false;

unsigned long lastPrint = 0;

// ---------------- OUTPUT DRIVERS ----------------
inline void scu_low() {
  digitalWrite(SCU_PIN, LOW);
}
inline void scu_high() {
  if (OUT_OPEN_DRAIN) {
    pinMode(SCU_PIN, INPUT_PULLUP);   // release: line returns to its pull-up (HIGH)
  } else {
    digitalWrite(SCU_PIN, HIGH);
  }
}

// ---------------- MATH ----------------
// real speed from pulse rate (period-based, no GPS anywhere)
inline float kmhFromHz(float hz) {
  float revsPerSec = hz / (float)IN_PULSES_PER_REV;
  return revsPerSec * (WHEEL_CIRC_MM / 1000.0f) * 3.6f;
}
inline float hzFromKmh(float kmh) {
  float revsPerSec = kmh / 3.6f / (WHEEL_CIRC_MM / 1000.0f);
  return revsPerSec * (float)OUT_PULSES_PER_REV;
}
// what speed the SCU should see, given real speed
// speedMode=true (magnet present) -> SPEED MODE; =false -> PASS
float policyKmh(float realKmh, bool speedMode) {
  if (speedMode && realKmh > RAMP_KNEE)
    return RAMP_SLOPE * realKmh + RAMP_INTERCEPT;   // SPEED MODE: scaled
  return realKmh;                                   // PASS
}

// ---------------- INPUT (main reed) ----------------
// speed from period between wheel pulses; light IIR for stability;
// eases to zero when the wheel stops
void handleInput(unsigned long now, unsigned long dt) {
  bool lvl    = (digitalRead(REED_PIN) == HIGH);
  bool active = REED_ACTIVE_LOW ? !lvl : lvl;        // true when magnet present

  if (active != activePrev) { activePrev = active; lastChange = now; }

  // count exactly once per magnet pass, after debounce
  if (activePrev && (now - lastChange) >= DEBOUNCE_US) {
    if (!countedThisPass && (now - lastPulseTime) >= MIN_PULSE_US) {
      if (havePulse) {
        unsigned long period  = now - lastPulseTime; // us per pulse
        float instKmh         = kmhFromHz(1000000.0f / (float)period);
        smoothKmh            += SPEED_ALPHA * (instKmh - smoothKmh);
      }
      havePulse       = true;
      lastPulseTime   = now;
      countedThisPass = true;
    }
  } else if (!activePrev) {
    countedThisPass = false;
  }

  // parked = 0: decay the estimate to zero after the wheel stops
  if (havePulse && (now - lastPulseTime) > STOP_TIMEOUT) {
    smoothKmh -= STOP_DECAY * ((float)dt / 1000000.0f);
    if (smoothKmh < 0.0f) smoothKmh = 0.0f;
  }
}

// ---------------- SAFETY REED (2nd reed) ----------------
// level-read + debounce. magnet present (reed CLOSED) => SPEED MODE;
// magnet absent (reed OPEN) => PASS (true speed, stock)
void handleKill(unsigned long now) {
  bool lvl = (digitalRead(KILL_PIN) == HIGH);
  bool raw = KILL_ACTIVE_LOW ? !lvl : lvl;           // true when magnet present

  if (raw != killRawPrev) { killRawPrev = raw; killChange = now; }
  if ((now - killChange) >= KILL_DEBOUNCE_US) killActive = raw;
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(KILL_PIN, INPUT_PULLUP);
  pinMode(SCU_PIN, OUTPUT);
  scu_high();                 // release the SCU line (HIGH via pull-up)
  lastLoop = micros();
  killChange = micros();
  if (USE_SERIAL) {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.print(F(FIRMWARE_REV));
    Serial.print(F(" Bosch reed interceptor ready."));
    if (USE_MODE_REED) {
      Serial.println(F("Safety reed: magnet present -> SPEED MODE (fast), magnet absent -> PASS (stock)"));
      Serial.println(F("Calibrate WHEEL_CIRC_MM with the safety magnet ABSENT."));
    } else {
      Serial.println(F("Safety reed DISABLED: always SPEED MODE."));
      Serial.println(F("Calibrate WHEEL_CIRC_MM."));
    }
  }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = micros();
  unsigned long dt  = now - lastLoop; lastLoop = now;
  if (dt > MAX_DT_US) dt = MAX_DT_US;   // no burst if the loop ever stalls

  handleInput(now, dt);
  handleKill(now);

  float realKmh   = smoothKmh;
  // USE_MODE_REED=true: safety reed picks the mode; =false: reed ignored, always fast
  bool speedMode = USE_MODE_REED ? killActive : false;
  float targetKmh = policyKmh(realKmh, speedMode);

  // slew-limit the reported speed toward the target: the 25-knee jump
  // becomes a smooth fake brake (up) / accel (down), not a step
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

  // finish the pulse low-time without blocking
  if (scuLow && now >= pulseLowUntil) { scu_high(); scuLow = false; }

  if (USE_SERIAL && (now - lastPrint >= 500000)) {
    lastPrint = now;
    Serial.print(F("real="));     Serial.print(realKmh, 1);
    Serial.print(F(" reported="));Serial.print(reportedKmh, 1);
    Serial.print(F(" safety="));  Serial.print(killActive ? F("true") : F("false"));
    Serial.println(F(" km/h"));
  }
}
