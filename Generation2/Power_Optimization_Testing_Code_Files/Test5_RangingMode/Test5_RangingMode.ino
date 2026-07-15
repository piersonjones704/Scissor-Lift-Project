/**
 * T05_ranging_mode.ino
 * ===========================================================================
 * TEST 05 — RANGING MODE: CONTINUOUS vs AUTONOMOUS
 * ===========================================================================
 * Variable changed : vl53l8cx_set_ranging_mode()
 * All other params : resolution = 8×8, frequency = 1 Hz,
 *                    integration = default (10 ms)
 *
 * Change TEST_MODE and reflash for each test:
 *   Test 11 (Continuous) : VL53L8CX_RANGING_MODE_CONTINUOUS
 *   Test 8  (Autonomous) : VL53L8CX_RANGING_MODE_AUTONOMOUS  (reference)
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Change this value and reflash for each test ────────────────────────── */
/* Options: VL53L8CX_RANGING_MODE_CONTINUOUS                                */
/*          VL53L8CX_RANGING_MODE_AUTONOMOUS                                */
#define TEST_MODE   VL53L8CX_RANGING_MODE_CONTINUOUS

/* ── Constant parameters ────────────────────────────────────────────────── */
#define TEST_FREQ_HZ   1

/* ── Globals ────────────────────────────────────────────────────name────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQ_HZ);
    vl53l8cx_set_ranging_mode(&Dev, TEST_MODE);

    /* Read back confirmed values */
    uint8_t  confirmed_mode;
    uint32_t confirmed_integ;
    vl53l8cx_get_ranging_mode(&Dev, &confirmed_mode);
    vl53l8cx_get_integration_time_ms(&Dev, &confirmed_integ);

    LOG("=== T05 RANGING MODE TEST ===");
    LOG("  Resolution  : 8x8 (64 zones)\n");
    LOGF("  Frequency   : %u Hz\n", TEST_FREQ_HZ);
    LOGF("  Mode        : %u (1=CONTINUOUS, 3=AUTONOMOUS)\n", confirmed_mode);
    LOGF("  Integration : %lu ms\n", confirmed_integ);
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