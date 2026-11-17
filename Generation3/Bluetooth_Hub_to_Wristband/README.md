# SafeLifts
 
A proximity warning system for a lift basket. Two sensor modules (front and back of the basket) measure distances to nearby objects and report them to a hub. The hub decides how urgent the situation is and tells a wristband worn by the operator, which shows the direction and distance on a small screen and buzzes faster as objects get closer.
 
**Status: working prototype on development boards. Not a certified safety product. Do not describe it as safety-rated.**
 
```
 FRONT MODULE ─┐
               ├─ BLE ─►  HUB (ESP32-S3)  ── BLE ─►  WRISTBAND (XIAO nRF52840)
 BACK MODULE  ─┘          merges modules,            screen + haptics
 (ESP32 + 4x ToF          decides urgency
  + 1x radar)
```
 
- One hub per lift, one wristband per hub (1:1 pairing is **not implemented yet**, see Known issues).
- The modules reduce their sensor data to a small per-sector report. The hub merges the two modules and makes the safety decision. The wristband only turns urgency into a buzz pattern and a screen.
## Repo layout
 
```
hub/SafeLifts_Hub_v0_3/            hub firmware (+ safelifts_module_packet.h)
wristband/SafeLifts_Wristband_OLED_v3_4/
module-emulator/SafeLifts_ModuleEmulator_v0_1/   (+ safelifts_module_packet.h)
docs/SafeLifts_SensorModule_Handoff_v1.md
archive/                           older versions
```
 
## Hardware
 
| Part | What |
|---|---|
| Wristband | Seeed XIAO nRF52840 Sense, Adafruit DRV2605L haptic driver, JYLRA1040ZF 10 mm coin LRA, GME12864-11 OLED (128x64, I2C 0x3C), 500 mAh LiPo. I2C on D4 (SDA) and D5 (SCL). The DRV2605L is at 0x5A. |
| Hub | ESP32-S3 dev board |
| Each sensor module | ESP32, 4x VL53L8CX multizone ToF, 1x IWRL6432 mmWave radar (**in progress**) |
 
Each module is meant to cover about 180 degrees horizontally and 80 degrees vertically (30 above, 50 below). Whether four ToF sensors plus the radar actually reach that is **not yet confirmed**.
 
## Software setup
 
Arduino IDE, with these installs.
 
**Wristband** (Seeed XIAO nRF52840 Sense)
- Board package: **Seeed nRF52 Boards** (Boards Manager). Select **Seeed XIAO nRF52840 Sense** (not the plain one).
- Libraries (Library Manager): **Adafruit SSD1306**, **Adafruit GFX Library**, **Adafruit DRV2605 Library**.
- Bluefruit and TinyUSB come with the board package. Do not install them separately.
**Hub and module emulator** (ESP32-S3)
- Board package: **esp32 by Espressif Systems** (Boards Manager). Select **ESP32S3 Dev Module**.
- *Tools → USB CDC On Boot* must be **Enabled**, or Serial output won't appear.
- Library: **NimBLE-Arduino by h2zero, version 2.3.2 or newer** (older 2.x versions had bugs applying connection limits).
- Open the Serial Monitor at **115200** with line ending set to **Newline**.
## Flashing and testing
 
1. Flash the wristband sketch to the XIAO. Boot output should include `I2C scan: 0x3C 0x5A`. If either address is missing, check wiring and power before anything else.
2. Flash the hub sketch to the ESP32-S3 (check that the board and port in *Tools* are the hub's before uploading).
3. Power both. The hub finds the wristband, connects, and the wristband shows **CLEAR**.
Hub Serial commands (simulated modules, on by default):
 
| Type | Expected on the wristband |
|---|---|
| `m0 f 50` | FRONT / 50 cm / WARNING, double-click buzz |
| `m1 b 30` | BACK / 30 cm / DANGER, rapid pulses |
| `reset`, then `m0 fault tofa` | SENSOR FAULT screen, slow triple click |
| `m0 off`, wait a second | SENSOR FAULT (a silent module is treated as a fault) |
| `pause` | LOST SIGNAL screen and long buzz within about 0.5 s |
| `pause` again | back to CLEAR |
 
Other commands: `sector names f fr r br b bl l fl`, `m0 multi f`, `status`, `help`, and `sim off` to switch to real modules only.
 
**Testing the real module link without sensors:** flash `SafeLifts_ModuleEmulator_v0_1` on a spare ESP32 (set `MODULE_ID` at the top of the sketch first), type `sim off` on the hub, and watch the hub connect and receive scripted reports. Use two boards to emulate both modules.
 
## Interfaces
 
**Sectors** (lift-relative, 45 degrees wide, clockwise from the lift front): 0 FRONT, 1 FRONT-RIGHT, 2 RIGHT, 3 BACK-RIGHT, 4 BACK, 5 BACK-LEFT, 6 LEFT, 7 FRONT-LEFT. Urgency: 0 clear, 1 caution, 2 warning, 3 danger.
 
**Hub to wristband** (6 bytes, Write Without Response, every 100 ms)
 
| Byte | Field | Notes |
|---|---|---|
| 0 | seq | increments every write |
| 1 | sector | 0 to 7, or 0xFF = clear |
| 2 | distance_cm | 255 = unknown |
| 3 | urgency | 0 to 3 |
| 4 | flags | bit0 = multiple objects, bit1 = sensor fault or degraded |
| 5 | reserved | 0 |
 
Service `f20270e0-4d91-4592-8d79-61117906f549`, Hazard State characteristic `095d4f3c-7f73-447f-bd75-97a0e4712903`.
 
**Module to hub** (15 bytes, notification, every 100 ms). The full definition is in `safelifts_module_packet.h`, which is the source of truth. Do not change the format without bumping its version and telling the team.
 
| Byte | Field |
|---|---|
| 0 | version (currently 2) |
| 1 | module_id (0 = FRONT, 1 = BACK) |
| 2 | seq |
| 3 | frame_period (10 ms units) |
| 4 | fault_flags (4 ToF, radar, battery, degraded) |
| 5 | multi_mask (one bit per sector) |
| 6 | moving_mask (reserved) |
| 7 to 14 | sector[0..7]: 0 to 250 = cm, 0xFF = clear, 0xFE = unknown, 0xFD = not covered |
 
Service `e18197a7-8972-47fc-80c2-4131fda76ac1`, Module Report characteristic `b57b1206-2eef-4394-824e-991501787caa` (notify).
 
## Fail-safe behavior (do not weaken)
 
- **Unknown is never clear.** A sector no working sensor can see is reported unknown, and the hub treats a silent module as unknown.
- **Silence is an alarm.** If the wristband gets no packet for 400 ms it shows an inverted **LOST SIGNAL** screen with a long buzz that matches no hazard pattern. This stays on until data returns.
- **A fault is the headline.** With no hazard but a fault flag, the wristband shows **SENSOR FAULT / CLEAR NOT CONFIRMED**, never a big CLEAR.
- A repeated identical module packet does not count as fresh.
## What has been verified
 
| Item | State |
|---|---|
| Hub v0.2 with simulated modules plus wristband v3.3 | Run on real hardware: hazards, faults, silent module, pause all behaved |
| Wristband v3.4 | Compiled and logic-tested on a computer stand-in only, **not yet flashed** |
| Hub v0.3 (real module receive path) | Connection and staleness logic tested on a simulated BLE world only. **Not compiled or run on real hardware.** Expect to fix compile errors if any NimBLE call differs |
| Module emulator v0.1 | **Not compiled or run on real hardware** |
| Real sensor modules | Not built yet |
 
## Known issues and limitations
 
- **No 1:1 pairing.** The hub connects to the first wristband it hears, and the wristband accepts writes from anyone. Two lifts on one site could cross-connect.
- **Hub v0.3 blocks while connecting a module** (up to about 2 s), so the wristband gets no updates and may show LOST SIGNAL during that time. Planned fix: a separate link-management task or asynchronous connect, after hardware tests.
- **The hub needs exactly 3 BLE connections** (wristband plus two modules), which is the NimBLE default, with no spare. Raising it needs a library config change.
- **Hazard thresholds are placeholders**: danger under 40 cm, warning under 80 cm, caution under 150 cm.
- **Ground returns:** looking downward from a raised basket, the sensors can see the ground and the lift's own structure. The module firmware must reject these without ever turning an unknown sector into clear.
- **Overhead objects are not treated as hazards** (decision for now: the operator can see them).
- **Charging and power-off:** the band is meant to be switched off at night to charge. If it merely sits on a charger powered on with the hub off, it will buzz LOST SIGNAL. The PCB must make "off" a real off, and the power switch placement must not cut the charging path (verify against the XIAO schematic).
- **No battery indicator**, and the wristband's current draw has not been measured against the 500 mAh cell.
- The lost-signal buzz never backs off. This is safe but may be tiring if someone walks away with the band on.
## Open decisions
 
- Real hazard distances (caution, warning, danger)
- Battery life target (one shift or a full day) and module power source
- Final module transport (BLE is implemented on the hub; alternatives like ESP-NOW would need hub changes)
- Whether the module reports more than nearest distance per sector (the packet is versioned so it can be extended)
## Working in this repo
 
- `git pull` before you start, and again before you push.
- Never commit passwords, Wi-Fi credentials, or SSH keys.
- If you change a packet or UUID, change it in **every** sketch and in `safelifts_module_packet.h`, bump the version, and tell the team.
- Sensor module work is described in `docs/SafeLifts_SensorModule_Handoff_v1.md`.
 
