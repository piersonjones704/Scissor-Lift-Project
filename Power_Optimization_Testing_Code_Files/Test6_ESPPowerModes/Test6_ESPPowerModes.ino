/**
 * Test13_ESP32LightSleep.ino
 * ===========================================================================
 * TEST 13 — ESP32 LIGHT SLEEP WITH INT PIN WAKE
 * ===========================================================================
 * Variable changed : ESP32 light sleep enabled between measurements
 * All other params : resolution = 8×8, frequency = 1 Hz, AUTONOMOUS 10 ms
 *
 * Approach
 *   1. Sensor ranges continuously and autonomously at 1 Hz
 *   2. When data is ready sensor pulses INT pin (GPIO 5)
 *   3. INT pin wakes ESP32 from light sleep
 *   4. ESP32 reads data via I2C
 *   5. ESP32 goes back to light sleep
 *   6. Repeat
 *
 * This is the most power efficient approach because:
 *   - Sensor never stops ranging
 *   - ESP32 sleeps for maximum time
 *   - ESP32 wakes exactly when data is ready
 *   - No ranging stop/start overhead
 *
 * Wiring required
 *   INT pin on Pololu board → GPIO 5 on ESP32
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

/* ── Constant parameters ────────────────────────────────────────────────── */
#define TEST_FREQ_HZ    1
#define INT_PIN         5    /* GPIO connected to sensor INT pin */
/* ── Toggle this to compare sleep vs no sleep ───────────────────────────── */
#define ENABLE_LIGHT_SLEEP   true   /* change to false to disable sleep */

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    delay(10);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQ_HZ);
    delay(10);
    vl53l8cx_set_integration_time_ms(&Dev, 10);
    delay(10);

    /* Configure INT pin as input */
    pinMode(INT_PIN, INPUT);

    /* Configure INT pin as wake source for light sleep */
    /* INT pin is active LOW on VL53L8CX — wakes on falling edge */
    esp_sleep_enable_ext1_wakeup(1ULL << INT_PIN, ESP_EXT1_WAKEUP_ANY_LOW);

    /* Read back confirmed values */
    uint8_t  confirmed_res, confirmed_freq, confirmed_mode;
    uint32_t confirmed_integ;
    vl53l8cx_get_resolution(&Dev, &confirmed_res);
    vl53l8cx_get_ranging_frequency_hz(&Dev, &confirmed_freq);
    vl53l8cx_get_ranging_mode(&Dev, &confirmed_mode);
    vl53l8cx_get_integration_time_ms(&Dev, &confirmed_integ);

    LOG("=== TEST 13 ESP32 LIGHT SLEEP + INT PIN WAKE ===");
    LOGF("  Resolution  : %u zones\n",          confirmed_res);
    LOGF("  Frequency   : %u Hz\n",             confirmed_freq);
    LOGF("  Mode        : %u (3=AUTONOMOUS)\n", confirmed_mode);
    LOGF("  Integration : %lu ms\n",            confirmed_integ);
    LOGF("  INT pin     : GPIO %u\n",           INT_PIN);
    LOG("[INFO]  Sensor ranging continuously");
    LOG("[INFO]  ESP32 sleeping between measurements");
    LOG("[INFO]  Monitor SYSTEM current on power meter");
    LOG("[INFO]  Sensor current on multimeter similar to Test 1");

    vl53l8cx_start_ranging(&Dev);
    delay(100);
}

void loop()
{
    uint8_t ready = 0;
    vl53l8cx_check_data_ready(&Dev, &ready);

    if (ready) {
        vl53l8cx_get_ranging_data(&Dev, &Results);
        print_result_summary(&Results, 64);
        Serial.flush();

        if (ENABLE_LIGHT_SLEEP) {
            esp_light_sleep_start();
        }
    } else {
        Serial.flush();
        if (ENABLE_LIGHT_SLEEP) {
            esp_light_sleep_start();
        }
    }
}