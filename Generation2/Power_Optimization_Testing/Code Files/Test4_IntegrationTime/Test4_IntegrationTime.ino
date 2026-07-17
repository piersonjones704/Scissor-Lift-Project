/**
 * T04_integration_time.ino
 * ===========================================================================
 * TEST 04 — INTEGRATION TIME
 * ===========================================================================
 * Variable changed : vl53l8cx_set_integration_time_ms()
 * All other params : resolution = 8×8, frequency = 1 Hz, AUTONOMOUS mode
 *
 * ULD limits (8×8, 1 Hz)
 *   Minimum integration time : 2 ms
 *   Maximum integration time : 1000 ms  (≤ 1/frequency)
 *   Default integration time : 10 ms
 *
 * To test each integration time:
 *   1. Change TEST_INTEGRATION_MS to desired value
 *   2. Flash to board
 *   3. Settle meter for 30-60 seconds
 *   4. Record I_min/I_avg/I_max
 *   5. Repeat for next value
 *
 * Suggested test values (below default of 10 ms):
 *   2, 3, 5, 7, 10 ms
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Change this value and reflash for each test ────────────────────────── */
/* Minimum: 2 ms   Maximum: 1000 ms at 1 Hz   Default: 10 ms              */
#define TEST_INTEGRATION_MS 7

/* ── Constant parameters ────────────────────────────────────────────────── */
#define TEST_FREQ_HZ   10

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQ_HZ);

    uint8_t status = vl53l8cx_set_integration_time_ms(&Dev, TEST_INTEGRATION_MS);
    if (status) {
        LOGF("[ERROR] set_integration_time_ms(%u) → status %u\n",
             TEST_INTEGRATION_MS, status);
    }

    /* Read back to confirm */
    uint32_t confirmed_integ;
    vl53l8cx_get_integration_time_ms(&Dev, &confirmed_integ);

    LOG("=== T04 INTEGRATION TIME TEST ===");
    LOG("  Resolution  : 8x8 (64 zones)\n");
    LOGF("  Frequency   : %u Hz\n",   TEST_FREQ_HZ);
    LOGF("  Integration : %lu ms\n",  confirmed_integ);
    LOG("[INFO]  Settle meter then record I_min/I_avg/I_max");
    LOG("[INFO]  Ranging indefinitely — reset board to stop");

    vl53l8cx_start_ranging(&Dev);
}

void loop()
{
    if (!wait_for_data(&Dev)) return;

    vl53l8cx_get_ranging_data(&Dev, &Results);
    print_result_summary(&Results, 64);
}