/**
 * BuzzerTest.ino
 * ===========================================================================
 * BUZZER CHARACTERIZATION TESTS
 * ===========================================================================
 * Tests buzzer behavior for power characterization
 * 
 * Test B1 — Continuous buzzer (always on)
 * Test B2 — Pulsed buzzer (beeping pattern)
 *
 * Change TEST_MODE and reflash for each test:
 *   BUZZER_CONTINUOUS  → B1
 *   BUZZER_PULSED      → B2
 *
 * Wiring:
 *   Buzzer positive → GPIO 10
 *   Buzzer negative → GND
 *
 * Measurement:
 *   Multimeter in series with buzzer power line
 *   Power meter for system total
 * ===========================================================================
 */

#include <Arduino.h>

/* ── Change this value and reflash for each test ────────────────────────── */
/* Options: BUZZER_CONTINUOUS or BUZZER_PULSED                              */
#define TEST_MODE   BUZZER_CONTINUOUS

/* ── Pin assignment ─────────────────────────────────────────────────────── */
#define BUZZER_PIN   1

/* ── Mode definitions ───────────────────────────────────────────────────── */
#define BUZZER_CONTINUOUS   0
#define BUZZER_PULSED       1

/* ── Pulsed mode timing ─────────────────────────────────────────────────── */
#define BEEP_ON_MS    2000   /* how long each beep lasts */
#define BEEP_OFF_MS   8000   /* silence between beeps */

void setup()
{
    Serial.begin(115200);
    delay(3000);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    if (TEST_MODE == BUZZER_CONTINUOUS) {
        Serial.println("=== B1 BUZZER CONTINUOUS ===");
        Serial.println("[INFO]  Buzzer on continuously");
        Serial.println("[INFO]  Settle meter then record I_min/I_avg/I_max");
        Serial.println("[INFO]  Reset board to stop");
        digitalWrite(BUZZER_PIN, HIGH);

    } else {
        Serial.println("=== B2 BUZZER PULSED ===");
        Serial.printf("  Beep on  : %u ms\n", BEEP_ON_MS);
        Serial.printf("  Beep off : %u ms\n", BEEP_OFF_MS);
        Serial.printf("  Duty cycle: %.1f%%\n",
            100.0f * BEEP_ON_MS / (BEEP_ON_MS + BEEP_OFF_MS));
        Serial.println("[INFO]  Settle meter then record I_min/I_avg/I_max");
        Serial.println("[INFO]  Reset board to stop");
    }
}

void loop()
{
    if (TEST_MODE == BUZZER_CONTINUOUS) {
        /* Nothing to do — buzzer stays on from setup() */

    } else {
        /* Pulsed beeping pattern */
        digitalWrite(BUZZER_PIN, HIGH);
        Serial.println("[BEEP]  ON");
        delay(BEEP_ON_MS);

        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("[BEEP]  OFF");
        delay(BEEP_OFF_MS);
    }
}