#include <AVATAR.h>

// Arm firmware WITH encoder feedback (based on Arm_anivik.c).
// The no-encoder version is arm_without_pid.c.
//
// Orin -> ESP : fields 2..7   per-motor PWM command, -255..255 (float)
// ESP -> Orin : fields 12..17 applied PWM echo (float)
//               fields 22..27 raw encoder counts (int)
//               fields 32..37 joint angle in degrees (float)
//               field  40     uptime in ms (int) -- lets the Orin detect an ESP reset
//
// Telemetry streams at a fixed TELEMETRY_PERIOD_MS whether or not commands
// arrive, so the Orin can always see the encoders (and can tell "encoders
// unplugged" apart from "no commands"). Motors still stop if no command
// arrives for WATCHDOG_TRIGGER_TIME_IN_S.

#define NUM_MOT 6
#define MAX_PWM 255
#define WATCHDOG_TRIGGER_TIME_IN_S 1
#define TELEMETRY_PERIOD_MS 10     // 100 Hz

// Base, Shoulder, Elbow, Bevel 1, Bevel 2, Gripper
int pwm_pins[NUM_MOT] = {2, 41, 6, 4, 15, 17};
int dir_pins[NUM_MOT] = {1, 42, 7, 5, 16, 18};
// Motor direction: flip an entry to 0 if positive PWM turns that motor the wrong way.
static const int dir_dir[NUM_MOT] = {1, 1, 1, 1, 1, 1};

// --- Encoders (ported from the old micro-ROS firmware) ---
// Base, Shoulder, Elbow, Bevel 1, Bevel 2, Gripper
int encA_pins[NUM_MOT] = {39, 48, 21, 14, 12, 10};
int encB_pins[NUM_MOT] = {40, 38, 47, 13, 11, 9};
// Encoder direction: flip an entry to -1 if positive PWM makes that encoder
// count DOWN. A reversed encoder makes the Orin's closed loop run away (the
// Orin flags this as a RUNAWAY fault). Check every motor in open-loop mode first.
static const int enc_dir[NUM_MOT] = {1, 1, 1, 1, 1, 1};

volatile int last_a[NUM_MOT] = {0};
volatile int last_b[NUM_MOT] = {0};
volatile long enc_count[NUM_MOT] = {0};

// Calibration constants carried over as-is from the old firmware -- these
// are measured values (counts per the stated degree range), not guesses.
// If any joint's mechanics changed since these were measured, recalibrate
// that entry specifically rather than assuming they still hold.
// TODO(verify): the gripper uses the same factor as the bevels -- confirm on the bench.
static const float enc_to_angle_factor[NUM_MOT] = {
    180.0f / 143082.5f,   // Base
    90.0f  / 390668.0f,   // Shoulder
    90.0f  / 64735.3f,    // Elbow
    90.0f  / 55157.0f,    // Bevel 1
    90.0f  / 55157.0f,    // Bevel 2
    90.0f  / 55157.0f     // Gripper
};

float target_data[NUM_MOT] = {0};   // raw PWM per motor, -255..255

uint32_t last_msg_time = 0;
uint32_t last_tel_time = 0;

AVATAR arm_link;

namespace Wire {
    const int32_t TARGET_START = 2;   // 2..7   = per-motor PWM command (float)
    const int32_t ECHO_START   = 12;  // 12..17 = applied PWM echo (float)
    const int32_t ENC_START    = 22;  // 22..27 = raw encoder counts (int) -- matches arm.py's enc1..6
    const int32_t ANGLE_START  = 32;  // 32..37 = derived joint angle, degrees (float) -- matches arm.py's angle1..6
    const int32_t UPTIME       = 40;  // 40     = millis() since boot (int) -- matches arm.py's uptime_ms
}

inline void drive_pwm(int i, float raw) {
    int val = (int)raw;
    int pwm = constrain(abs(val), 0, MAX_PWM);
    int dir = (val >= 0) ? dir_dir[i] : !dir_dir[i];
    digitalWrite(dir_pins[i], dir);
    analogWrite(pwm_pins[i], pwm);
}

// ---- Quadrature decode, ported from the old firmware ----
// IRAM_ATTR: called from the ISRs, so it must not live in flash.
//
// Every edge on every encoder pin is an interrupt. At high joint speed (the
// shoulder is ~4300 counts/deg) this can exceed what the ESP32 can service
// and counts get lost silently. If a fast move followed by a slow return
// doesn't read the same angle, move the busiest encoders to the ESP32-S3's
// hardware pulse counter (PCNT, 4 units).

void IRAM_ATTR updateEncoder(int i) {
    int a = digitalRead(encA_pins[i]);
    int b = digitalRead(encB_pins[i]);

    int last_a_state = last_a[i];
    int last_b_state = last_b[i];

    if (a != last_a_state || b != last_b_state) {
        if (last_a_state == a) {
            enc_count[i] += (a ^ b) ? -1 : +1;
        } else {
            enc_count[i] += (a ^ b) ? +1 : -1;
        }
        last_a[i] = a;
        last_b[i] = b;
    }
}

void IRAM_ATTR isr0A() {updateEncoder(0);}
void IRAM_ATTR isr0B() {updateEncoder(0);}
void IRAM_ATTR isr1A() {updateEncoder(1);}
void IRAM_ATTR isr1B() {updateEncoder(1);}
void IRAM_ATTR isr2A() {updateEncoder(2);}
void IRAM_ATTR isr2B() {updateEncoder(2);}
void IRAM_ATTR isr3A() {updateEncoder(3);}
void IRAM_ATTR isr3B() {updateEncoder(3);}
void IRAM_ATTR isr4A() {updateEncoder(4);}
void IRAM_ATTR isr4B() {updateEncoder(4);}
void IRAM_ATTR isr5A() {updateEncoder(5);}
void IRAM_ATTR isr5B() {updateEncoder(5);}

void setupEncoders() {
    for (int i = 0; i < NUM_MOT; i++) {
        pinMode(encA_pins[i], INPUT_PULLUP);
        pinMode(encB_pins[i], INPUT_PULLUP);
        last_a[i] = digitalRead(encA_pins[i]);
        last_b[i] = digitalRead(encB_pins[i]);
    }
    attachInterrupt(digitalPinToInterrupt(encA_pins[0]), isr0A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[0]), isr0B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[1]), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[1]), isr1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[2]), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[2]), isr2B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[3]), isr3A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[3]), isr3B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[4]), isr4A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[4]), isr4B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_pins[5]), isr5A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB_pins[5]), isr5B, CHANGE);
}

// ---- AVATAR link ----

void transmit_telemetry() {
    // Snapshot all counts together so the six angles describe one instant.
    long counts[NUM_MOT];
    noInterrupts();
    for (int i = 0; i < NUM_MOT; i++)
        counts[i] = enc_count[i];
    interrupts();

    arm_link.clearTx();
    for (int i = 0; i < NUM_MOT; i++)
        arm_link.addFloat(Wire::ECHO_START + i, target_data[i]);
    for (int i = 0; i < NUM_MOT; i++) {
        long count = enc_dir[i] * counts[i];
        arm_link.addInt(Wire::ENC_START + i, (int32_t)count);
        arm_link.addFloat(Wire::ANGLE_START + i, count * enc_to_angle_factor[i]);
    }
    arm_link.addInt(Wire::UPTIME, (int64_t)millis());
    arm_link.sendTx();
}

void arm_callback() {
    last_msg_time = millis();
    for (int i = 0; i < NUM_MOT; i++) {
        int id = Wire::TARGET_START + i;
        if (arm_link.hasField(id))
            target_data[i] = arm_link.getFloat(id);
    }
    // Apply immediately (instead of waiting for the next loop tick) to keep
    // command latency low. Telemetry is NOT sent from here: it goes out from
    // loop() at a fixed rate, so the Orin's velocity estimate sees even spacing.
    for (int i = 0; i < NUM_MOT; i++)
        drive_pwm(i, target_data[i]);
}

void setup() {
    for (int i = 0; i < NUM_MOT; i++) {
        pinMode(pwm_pins[i], OUTPUT);
        pinMode(dir_pins[i], OUTPUT);
        analogWrite(pwm_pins[i], 0);
        digitalWrite(dir_pins[i], dir_dir[i]);
    }
    setupEncoders();
    arm_link.begin(115200);
    arm_link.setCallback(arm_callback);
    last_msg_time = millis();
    last_tel_time = millis();
}

void loop() {
    arm_link.update();
    uint32_t now = millis();

    if (now - last_tel_time >= TELEMETRY_PERIOD_MS) {
        last_tel_time = now;

        if (now - last_msg_time < (WATCHDOG_TRIGGER_TIME_IN_S * 1000)) {
            for (int i = 0; i < NUM_MOT; i++)
                drive_pwm(i, target_data[i]);
            neopixelWrite(RGB_BUILTIN, 0, 100, 0);    // green: commands arriving
        } else {
            for (int i = 0; i < NUM_MOT; i++) {
                target_data[i] = 0;
                drive_pwm(i, 0);
            }
            neopixelWrite(RGB_BUILTIN, 150, 0, 0);    // red: watchdog stop
        }

        // Always stream encoders, even while stopped by the watchdog.
        transmit_telemetry();
    }
}
