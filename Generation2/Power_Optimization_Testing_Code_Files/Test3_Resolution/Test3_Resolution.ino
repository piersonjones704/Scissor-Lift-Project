/**
 * T03_resolution.ino
 * ===========================================================================
 * TEST 03 — RESOLUTION COMPARISON  (4×4 vs 8×8)
 * ===========================================================================
 * Variable changed : vl53l8cx_set_resolution()
 * All other params : frequency = 1 Hz (same as T01 baseline)
 *
 * What changes internally
 *   8×8 mode activates 64 zones instead of 16.  The sensor must process
 *   4× more SPAD data per frame.  This increases:
 *     - Active computation time per frame
 *     - I2C data transfer size (roughly 4×)
 *     - Average current during the active portion of each cycle
 *
 * Frequency limits imposed by the ULD
 *   4×4 : 1–60 Hz
 *   8×8 : 1–15 Hz   ← enforced by the API; setting > 15 Hz returns error
 *
 * Expected power impact
 *   Moderate increase: +5–15 mA average at 1 Hz.
 *   The active window is longer but the overall duty-cycle at 1 Hz is low,
 *   so the effect on average current is real but not dramatic.
 *
 * Log two rows in your power table:
 *   T03a  4×4, 1 Hz
 *   T03b  8×8, 1 Hz
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Test parameters ────────────────────────────────────────────────────── */
#define TEST_FREQUENCY_HZ   10

/* ── Change this line and reflash for each test ─────────────────────────── */
/* Options: VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8            */
#define TEST_RESOLUTION     VL53L8CX_RESOLUTION_4X4

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration  Dev;
static VL53L8CX_ResultsData    Results;

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, TEST_RESOLUTION);
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQUENCY_HZ);

    uint8_t resolution;
    vl53l8cx_get_resolution(&Dev, &resolution);

    LOG("=== T03 RESOLUTION TEST ===\n");
    LOGF("  Resolution  : %s (%u zones)\n",
         (TEST_RESOLUTION == VL53L8CX_RESOLUTION_4X4) ? "4x4" : "8x8",
         resolution);
    LOGF("  Frequency   : %u Hz\n", TEST_FREQUENCY_HZ);
    LOG("[INFO]  Settle meter then record I_min/I_avg/I_max");
    LOG("[INFO]  Ranging indefinitely — reset board to stop");

    vl53l8cx_start_ranging(&Dev);
}

void loop()
{
    if (!wait_for_data(&Dev)) return;

    vl53l8cx_get_ranging_data(&Dev, &Results);

    uint8_t nb_zones = (TEST_RESOLUTION == VL53L8CX_RESOLUTION_4X4) ? 16 : 64;
    print_result_summary(&Results, nb_zones);
}