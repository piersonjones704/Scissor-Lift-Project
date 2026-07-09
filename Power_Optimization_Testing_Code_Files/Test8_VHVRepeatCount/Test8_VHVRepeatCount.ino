/**
 * Test8_VHVRepeatCount.ino
 * ===========================================================================
 * TEST 08 — VHV (VCSEL HIGH VOLTAGE) REPEAT COUNT REDUCTION
 * ===========================================================================
 * Variable changed : vl53l8cx_set_VHV_repeat_count()
 * All other params : resolution = 8×8, frequency = 10 Hz, AUTONOMOUS 10 ms
 *
 * What VHV calibration does
 *   The sensor runs an internal VHV calibration routine periodically to
 *   compensate for VCSEL bias drift with temperature. Each calibration
 *   extends the active period of that frame. Reducing how often it runs
 *   lowers average current at the cost of slightly slower thermal tracking.
 *
 * ULD default repeat count: confirmed from hardware readback before setting
 * Range: 1 – 255  (0 = disabled, runs only at init)
 *
 * To test each VHV repeat count:
 *   1. Change TEST_VHV_COUNT to desired value
 *   2. Flash to board
 *   3. Settle meter for 30-60 seconds
 *   4. Record I_min/I_avg/I_max
 *   5. Repeat for next value
 *
 * Test values:
 *   1   → calibrate every frame (default)
 *   10  → calibrate every 10th frame  (Test 14)
 *   40  → calibrate every 40th frame  (Test 15)
 *   255 → calibrate once at init only (Test 16)
 *
 * Thermal drift consideration
 *   In a scissor-lift environment with motor heat and varying ambient
 *   temperature, setting repeat count > 100 may cause VHV drift.
 *   A value of 10–40 is a reasonable operational compromise.
 *
 * NOTE: Multimeter should NOT be in series with sensor VIN during testing
 *   as the voltage drop causes start_ranging() to fail intermittently.
 *   Use power meter for system total current measurement only.
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Change this value and reflash for each test ────────────────────────── */
/* Options : 1 (default), 10, 40, 255                                       */
#define TEST_VHV_COUNT   10

/* ── Constant parameters ────────────────────────────────────────────────── */
#define TEST_FREQ_HZ   10

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    /* Read VHV default BEFORE setting anything */
    uint32_t default_vhv;
    vl53l8cx_get_VHV_repeat_count(&Dev, &default_vhv);
    LOGF("VHV default after init: %lu\n", default_vhv);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    delay(10);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQ_HZ);
    delay(10);
    vl53l8cx_set_integration_time_ms(&Dev, 10);
    delay(10);

    uint8_t status = vl53l8cx_set_VHV_repeat_count(&Dev, TEST_VHV_COUNT);
    if (status) {
        LOGF("[ERROR] set_VHV_repeat_count(%u) → status %u\n", TEST_VHV_COUNT, status);
    }
    delay(100);

    /* Read back to confirm */
    uint32_t confirmed_vhv;
    vl53l8cx_get_VHV_repeat_count(&Dev, &confirmed_vhv);

    LOG("=== T08 VHV REPEAT COUNT TEST ===");
    LOG("  Resolution  : 8x8 (64 zones)");
    LOGF("  Frequency   : %u Hz\n",  TEST_FREQ_HZ);
    LOG("  Mode        : AUTONOMOUS");
    LOG("  Integration : 10 ms");
    LOGF("  VHV count   : %lu\n",    confirmed_vhv);
    LOG("[INFO]  Settle meter then record I_min/I_avg/I_max");
    LOG("[INFO]  Ranging indefinitely — reset board to stop");

    vl53l8cx_start_ranging(&Dev);
    delay(500);
}

void loop()
{
    if (!wait_for_data(&Dev)) return;

    vl53l8cx_get_ranging_data(&Dev, &Results);
    print_result_summary(&Results, 64);
}