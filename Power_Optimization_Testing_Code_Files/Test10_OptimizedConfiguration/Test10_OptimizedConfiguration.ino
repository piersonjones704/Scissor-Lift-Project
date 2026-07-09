/**
 * T010_optimised_config.ino
 * ===========================================================================
 * TEST 010 — OPTIMISED COMBINED CONFIGURATION
 * ===========================================================================
 * Purpose
 *   Apply the best settings discovered in T01–T08 simultaneously and
 *   measure mAh / mWh over 30 minutes via inline USB power meter.
 *
 * Settings applied
 *   Resolution  : 4×4
 *   Frequency   : 1 Hz (fixed — consistent across all Table 10 runs)
 *   Mode        : AUTONOMOUS
 *   Integration : 10 ms
 *   VHV repeat  : 10  (calibrate every 10th frame)
 *   Target order: CLOSEST
 *   ESP32 sleep : Light sleep between frames — wake on INT pin LOW (GPIO 5)
 *   WiFi / BT   : OFF
 *   Buzzer      : Non-blocking state machine, direct profile assignment
 *
 * Test procedure
 *   Run A — No object present for 30 min  → idle power baseline
 *   Run B — Object held at ~150 cm for 30 min → active power with buzzer
 *   Reset USB power meter between runs.
 *   Weighted average (20% detection assumption):
 *     I_avg = (Run_A_mAh × 0.80) + (Run_B_mAh × 0.20)
 *
 * Wiring
 *   SDA       → GPIO 8
 *   SCL       → GPIO 9
 *   SPI/I2C   → GND   (CRITICAL — selects I2C mode)
 *   LP pin    → 3.3V
 *   INT       → GPIO 5
 *   Buzzer+   → 3.3V
 *   Buzzer-   → NPN transistor collector
 *   Transistor base → 1 kΩ → GPIO 1
 *   Transistor emitter → GND
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

/* ── Pin assignments ────────────────────────────────────────────────────── */
#define BUZZER_PIN      1
#define INT_PIN         5
#define INT_GPIO        GPIO_NUM_5

/* ── Sensor configuration ───────────────────────────────────────────────── */
#define VHV_REPEAT      10
#define RANGING_FREQ    1
#define INTEGRATION_MS  10
#define NUM_ZONES       16          // 4×4 = 16 zones (not 64)

/* ── Detection thresholds (mm) ──────────────────────────────────────────── */
#define DIST_VERY_CLOSE  500
#define DIST_CLOSE      1000
#define DIST_MEDIUM     1500
#define DIST_FAR        2000

/* ── Beep timing profiles (on_ms, off_ms) ──────────────────────────────── */
struct BeepProfile {
    uint32_t on_ms;
    uint32_t off_ms;
};

static const BeepProfile PROFILES[] = {
    {  0,    0},    // 0: silent    (>200 cm)
    {800, 1200},    // 1: slow      (150–200 cm)
    {500,  500},    // 2: medium    (100–150 cm)
    {200,  200},    // 3: fast      (50–100 cm)
    {100,  100},    // 4: very fast (<50 cm)
};

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

static uint8_t  currentProfile   = 0;
static uint8_t  buzzerState      = 0;       // 0 = off, 1 = on
static uint32_t lastBuzzerChange = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * get_profile
 * Map a distance in mm to a buzzer profile index.
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint8_t get_profile(int16_t dist_mm)
{
    if (dist_mm <= 0 || dist_mm > DIST_FAR)  return 0;
    if (dist_mm > DIST_MEDIUM)               return 1;
    if (dist_mm > DIST_CLOSE)                return 2;
    if (dist_mm > DIST_VERY_CLOSE)           return 3;
    return 4;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * update_buzzer
 * Non-blocking buzzer state machine.
 * Called once per frame (once per second at 1 Hz).
 * millis() is RTC-backed on ESP32-S3 and continues during light sleep,
 * so elapsed time is accurate across sleep periods.
 * Buzzer GPIO state is preserved during light sleep — it holds HIGH or LOW
 * between wake events without any code running.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void update_buzzer()
{
    if (currentProfile == 0) {
        if (buzzerState == 1) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerState = 0;
        }
        return;
    }

    uint32_t now     = millis();
    uint32_t elapsed = now - lastBuzzerChange;
    uint32_t on_ms   = PROFILES[currentProfile].on_ms;
    uint32_t off_ms  = PROFILES[currentProfile].off_ms;

    if (buzzerState == 1 && elapsed >= on_ms) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState      = 0;
        lastBuzzerChange = now;
    } else if (buzzerState == 0 && elapsed >= off_ms) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState      = 1;
        lastBuzzerChange = now;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * get_closest_distance
 * Scan all 16 zones (4×4) and return nearest valid target in mm.
 * Status 5 = valid ranging result.
 * Returns 0 if no valid target found.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int16_t get_closest_distance()
{
    int16_t closest = 32767;
    for (uint8_t z = 0; z < NUM_ZONES; z++) {
        uint8_t idx = z * VL53L8CX_NB_TARGET_PER_ZONE;
        if (Results.target_status[idx] == 5 && Results.distance_mm[idx] > 0) {
            if (Results.distance_mm[idx] < closest) {
                closest = Results.distance_mm[idx];
            }
        }
    }
    return (closest == 32767) ? 0 : closest;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * setup
 * ═══════════════════════════════════════════════════════════════════════════ */
void setup()
{
    if (!sensor_init(&Dev)) {
        while (1) delay(1000);
    }

    vl53l8cx_set_resolution          (&Dev, VL53L8CX_RESOLUTION_4X4);
    delay(10);
    vl53l8cx_set_ranging_mode        (&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev, RANGING_FREQ);
    delay(10);
    vl53l8cx_set_integration_time_ms (&Dev, INTEGRATION_MS);
    delay(10);
    vl53l8cx_set_target_order        (&Dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    delay(10);
    vl53l8cx_set_VHV_repeat_count    (&Dev, VHV_REPEAT);
    delay(10);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // INT pin — sensor pulls LOW when data is ready
    // Level-triggered GPIO wake keeps light sleep responsive at 1 Hz
    pinMode(INT_PIN, INPUT_PULLUP);
    gpio_wakeup_enable(INT_GPIO, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    uint8_t  res, freq, mode;
    uint32_t integ, vhv;
    vl53l8cx_get_resolution          (&Dev, &res);
    vl53l8cx_get_ranging_frequency_hz(&Dev, &freq);
    vl53l8cx_get_ranging_mode        (&Dev, &mode);
    vl53l8cx_get_integration_time_ms (&Dev, &integ);
    vl53l8cx_get_VHV_repeat_count    (&Dev, &vhv);

    LOG("=== TABLE 10 — OPTIMISED ENERGY CONSUMPTION ===");
    LOGF("  Resolution  : %u zones (4x4)\n",       res);
    LOGF("  Frequency   : %u Hz\n",                freq);
    LOGF("  Mode        : %u (3=AUTONOMOUS)\n",    mode);
    LOGF("  Integration : %lu ms\n",               integ);
    LOGF("  VHV repeat  : %lu (every %lu frames)\n", vhv, vhv);
    LOG ("  Target order: CLOSEST");
    LOG ("  WiFi / BT   : Off");
    LOG ("  ESP32 sleep : Light sleep — wake on INT GPIO 5 LOW");
    LOG ("  Buzzer      : Active (proximity logic enabled)");
    LOG ("");
    LOG ("[INFO]  Note starting mAh on power meter NOW");
    LOG ("[INFO]  Run A: no object present for 30 min  (idle baseline)");
    LOG ("[INFO]  Run B: object at ~150 cm for 30 min  (active + buzzer)");
    LOG ("[INFO]  Reset power meter between runs");
    LOG ("================================================");

    vl53l8cx_start_ranging(&Dev);
    delay(100);

    // Initialise buzzer timer to now so the first elapsed reading is
    // clean rather than a large stale value from millis() at boot.
    lastBuzzerChange = millis();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * loop
 * ═══════════════════════════════════════════════════════════════════════════ */
void loop()
{
    // Sleep until INT goes LOW (data ready). Guard skips sleep if INT is
    // already LOW (frame fired while we were processing the previous one).
    if (digitalRead(INT_PIN) == HIGH) {
        esp_light_sleep_start();
    }

    uint8_t data_ready = 0;
    vl53l8cx_check_data_ready(&Dev, &data_ready);
    if (!data_ready) {
        // Spurious wake — go back to sleep
        return;
    }

    vl53l8cx_get_ranging_data(&Dev, &Results);

    int16_t closest_mm = get_closest_distance();

    // Direct profile assignment — no stepping for this test sketch.
    // Smooth single-step transitions belong in the final production code.
    currentProfile = get_profile(closest_mm);

    update_buzzer();

    const char *label;
    switch (currentProfile) {
        case 0:  label = "SILENT";     break;
        case 1:  label = "SLOW";       break;
        case 2:  label = "MEDIUM";     break;
        case 3:  label = "FAST";       break;
        case 4:  label = "VERY FAST";  break;
        default: label = "UNKNOWN";    break;
    }

    LOGF("[DETECT] dist=%d mm  profile=%s  buzzer=%s\n",
         closest_mm, label, buzzerState ? "ON" : "OFF");
}