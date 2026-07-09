/**
 * Table9_Baseline_EnergyConsumption.ino
 * ===========================================================================
 * TABLE 9 — BASELINE ENERGY CONSUMPTION TEST
 * ===========================================================================
 * Configuration: Default parameters (Test ID 1) + proximity buzzer logic
 *
 * Sensor parameters (default baseline):
 *   Resolution  : 8x8
 *   Frequency   : 1 Hz
 *   Integration : 10 ms (default)
 *   Mode        : Autonomous
 *   VHV         : Natural default
 *   WiFi        : Off
 *   ESP Sleep   : Off
 *
 * Buzzer logic:
 *   Object > 200 cm  : silent
 *   Object 150-200 cm: slow beep (800ms on, 1200ms off)
 *   Object 100-150 cm: medium beep (500ms on, 500ms off)
 *   Object 50-100 cm : fast beep (200ms on, 200ms off)
 *   Object < 50 cm   : very fast beep (100ms on, 100ms off)
 *
 * Measurement procedure:
 *   1. Note starting mAh on power meter
 *   2. Run for 30 minutes minimum
 *   3. Record mAh consumed
 *   4. Calculate estimated 10hr runtime
 *
 * Wiring:
 *   Sensor : SDA=GPIO8, SCL=GPIO9, SPI/I2C=GND, LP=3.3V
 *   Buzzer : positive=3.3V, negative=transistor collector
 *   Transistor base : 1kΩ resistor → GPIO 1
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Pin assignment ─────────────────────────────────────────────────────── */
#define BUZZER_PIN   1

/* ── Detection thresholds (mm) ──────────────────────────────────────────── */
#define DIST_VERY_CLOSE    500    /*  < 50 cm  → very fast beep */
#define DIST_CLOSE        1000    /*  < 100 cm → fast beep      */
#define DIST_MEDIUM       1500    /*  < 150 cm → medium beep    */
#define DIST_FAR          2000    /*  < 200 cm → slow beep      */
                                  /* >= 200 cm → silent         */

/* ── Beep timing profiles (on_ms, off_ms) ──────────────────────────────── */
struct BeepProfile {
    uint32_t on_ms;
    uint32_t off_ms;
};

static const BeepProfile PROFILES[] = {
    {0,    0},      /* 0: silent     (>200 cm)    */
    {800,  1200},   /* 1: slow       (150-200 cm) */
    {500,  500},    /* 2: medium     (100-150 cm) */
    {200,  200},    /* 3: fast       (50-100 cm)  */
    {100,  100},    /* 4: very fast  (<50 cm)     */
};

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

/* ── Buzzer state machine ───────────────────────────────────────────────── */
static uint8_t  currentProfile    = 0;
static uint8_t  buzzerState       = 0;      /* 0=off, 1=on */
static uint32_t lastBuzzerChange  = 0;

/**
 * @brief Determine beep profile based on distance
 */
static uint8_t get_profile(int16_t dist_mm)
{
    if (dist_mm <= 0 || dist_mm > DIST_FAR)    return 0;  /* silent */
    if (dist_mm > DIST_MEDIUM)                  return 1;  /* slow */
    if (dist_mm > DIST_CLOSE)                   return 2;  /* medium */
    if (dist_mm > DIST_VERY_CLOSE)              return 3;  /* fast */
    return 4;                                              /* very fast */
}

/**
 * @brief Update buzzer state machine — call every loop iteration
 *        Uses non-blocking timing so ranging is never delayed
 */
static void update_buzzer()
{

    /* Silent mode — turn off and return */
    if (currentProfile == 0) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = 0;
        return;
    }

    uint32_t now     = millis();
    uint32_t elapsed = now - lastBuzzerChange;
    uint32_t on_ms   = PROFILES[currentProfile].on_ms;
    uint32_t off_ms  = PROFILES[currentProfile].off_ms;

    if (buzzerState == 1 && elapsed >= on_ms) {
        /* Time to turn off */
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = 0;
        lastBuzzerChange = now;
    } else if (buzzerState == 0 && elapsed >= off_ms) {
        /* Time to turn on */
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = 1;
        lastBuzzerChange = now;
    }
}

/**
 * @brief Get the closest valid distance across all zones
 */
static int16_t get_closest_distance()
{
    int16_t closest = 32767;
    for (uint8_t i = 0; i < 64; i++) {
        if (Results.target_status[i] == 5 && Results.distance_mm[i] > 0) {
            if (Results.distance_mm[i] < closest) {
                closest = Results.distance_mm[i];
            }
        }
    }
    return (closest == 32767) ? 0 : closest;
}

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    /* Default baseline configuration — no changes from Test 1 */
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    delay(10);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 1);
    delay(10);
    vl53l8cx_set_target_order(&Dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    delay(10);

    /* Buzzer setup */
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    /* Print confirmed configuration */
    uint8_t  res, freq, mode;
    uint32_t integ;
    vl53l8cx_get_resolution(&Dev, &res);
    vl53l8cx_get_ranging_frequency_hz(&Dev, &freq);
    vl53l8cx_get_ranging_mode(&Dev, &mode);
    vl53l8cx_get_integration_time_ms(&Dev, &integ);

    LOG("=== TABLE 9 BASELINE ENERGY CONSUMPTION ===");
    LOGF("  Resolution  : %u zones\n",          res);
    LOGF("  Frequency   : %u Hz\n",             freq);
    LOGF("  Mode        : %u (3=AUTONOMOUS)\n", mode);
    LOGF("  Integration : %lu ms\n",            integ);
    LOG("  WiFi        : Off");
    LOG("  ESP Sleep   : Off");
    LOG("  Buzzer      : Active (proximity logic enabled)");
    LOG("");
    LOG("[INFO]  Note starting mAh on power meter NOW");
    LOG("[INFO]  Run for minimum 30 minutes");
    LOG("[INFO]  Buzzer beeps when object detected within 200 cm");
    LOG("[INFO]  Beep rate increases as object gets closer");
    LOG("==========================================");

    vl53l8cx_start_ranging(&Dev);
    delay(100);
}

void loop()
{
    /* Update buzzer state machine every iteration — non blocking */
    update_buzzer();

    /* Check if ranging data is ready */
    uint8_t ready = 0;
    vl53l8cx_check_data_ready(&Dev, &ready);

    if (ready) {
        vl53l8cx_get_ranging_data(&Dev, &Results);

        /* Get closest valid target across all zones */
        int16_t closest_mm = get_closest_distance();

        /* Update beep profile based on distance */
        uint8_t newProfile = get_profile(closest_mm);

        LOGF("[DEBUG] dist=%d mm  profile=%u  buzzerState=%u\n", // Debug lines
         closest_mm, newProfile, buzzerState); // Debug lines


        /* Smooth transition — only update profile if it changed */
        if (newProfile != currentProfile) {
            currentProfile = newProfile;
            /* Reset buzzer timing on profile change for smooth transition */
            lastBuzzerChange = millis();
            if (currentProfile > 0) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerState = 1;
            }
        }

        /* Log data */
        const char *zoneLabel;
        switch (currentProfile) {
            case 0: zoneLabel = "SILENT";    break;
            case 1: zoneLabel = "SLOW";      break;
            case 2: zoneLabel = "MEDIUM";    break;
            case 3: zoneLabel = "FAST";      break;
            case 4: zoneLabel = "VERYFAST";  break;
            default: zoneLabel = "UNKNOWN";  break;
        }

        LOGF("[DETECT] dist=%d mm  profile=%s  buzzer=%s\n",
             closest_mm,
             zoneLabel,
             buzzerState ? "ON" : "OFF");
        LOGF("[BUZZER] Pin state: %d\n", digitalRead(BUZZER_PIN));
    }
}