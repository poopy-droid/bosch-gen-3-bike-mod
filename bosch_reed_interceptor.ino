/*
 * Bosch eBike 25 km/h limiter interceptor
 * Wired inline in the existing rear reed -> SCU cable. ONLY the reed is tapped.
 * Firmware v1.3
 *
 * Full docs (modes, safety reed, slew, wiring, limits): docs.txt
 */

#define FIRMWARE_REV "v1.3"

// ---------------- CONFIG ----------------
#define REED_PIN      2     // rear reed signal
#define KILL_PIN      4     // safety reed signal (magnet near = SPEED MODE)
#define SCU_PIN       3     // output to the SCU

// true = line goes LOW when magnet present; false = goes HIGH
// (Most 2-wire reed-to-GND setups are LOW-active. Test & flip if wrong.)
#define REED_ACTIVE_LOW  true
#define KILL_ACTIVE_LOW  true

// wheel+tire circumference in mm
#define WHEEL_CIRC_MM     2100.0f
// magnets on the wheel (usually 1); fake pulses emitted per rev (keep 1)
#define IN_PULSES_PER_REV   1
#define OUT_PULSES_PER_REV  1

// ---------------- POLICY ----------------
// safety reed ON  (true)  : magnet near -> SPEED MODE, magnet away -> PASS
// safety reed OFF (false) : always SPEED MODE (no magnet needed)
#define USE_MODE_REED true

// SPEED MODE: above the knee, reported = 0.1*real + 10 (e.g. 56 -> 15.6);
// stays < 25 for real < 150, so support never cuts
#define RAMP_KNEE      25.0f  // reported == real at/below this
#define RAMP_SLOPE     0.1f
#define RAMP_INTERCEPT 10.0f

// base output pulse width (auto-clamped to period/3)
#define OUT_PULSE_WIDTH_US 3000

// ---------------- SLEW ----------------
// max rate the reported speed may change (km/h per second)
#define SLEW_KMH_PER_S 12.0f

#define USE_SERIAL   true
#define SERIAL_BAUD  115200

// ---------------- INTERNALS (tweak rarely) ----------------
#define SPEED_ALPHA      0.5f     // IIR on the period speed
#define STOP_TIMEOUT     1500UL   // ease speed to 0 when parked (us)
#define STOP_DECAY       30.0f    // km/h per second the estimate decays once stopped
#define DEBOUNCE_US      3000UL   // input edge debounce (us)
#define KILL_DEBOUNCE_US 50000UL  // safety reed level debounce (us)
#define MIN_PULSE_US     20000UL  // ignore input edges closer than this
#define MAX_OUT_HZ       40.0     // safety clamp on emitted rate
#define MAX_DT_US        10000UL  // clamp loop dt so a stall can't fire a burst

// true = open-drain (true reed mimic), false = push-pull
#define OUT_OPEN_DRAIN   true

// ---------------- STATE ----------------
// speed estimator: period between wheel pulses + light IIR
float  smoothKmh = 0.0f;         // smoothed REAL speed (km/h)
bool   havePulse = false;
bool   countedThisPass = false;
bool   activePrev = false;
unsigned long lastChange = 0;
unsigned long lastPulseTime = 0; // micros() of last counted pulse

float  reportedKmh = 0.0f;   // speed we actually emit (slew-limited)
double outPhase    = 0.0;
unsigned long lastLoop = 0;
bool   scuLow      = false;
unsigned long pulseLowUntil = 0;

// safety reed state (debounced): true = magnet present
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
// level-read + debounce; magnet present => true (SPEED MODE)
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
    } else {
      Serial.println(F("Safety reed DISABLED: always SPEED MODE."));
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

  // slew-limit the reported speed toward the target (why: see docs.txt)
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
