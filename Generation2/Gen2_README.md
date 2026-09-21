# Scissor Lift Collision Prevention Module
## Generation 2 - Time of Flight Sensors

### Overview  
This project, developed in collaboration with **Skanska**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.  

Generation 2 is a work in progress. The goal is to replace Generation 1's single mechanically-swept ultrasonic module with a static array of  6 VL53L8CX time-of-flight sensors, each mounted at a different fixed angle around the lift — eliminating the need for a servo sweep entirely. 

This semester, single-sensor operation was validated end-to-end, including adaptive ranging and a power-optimized configuration. Reliably running all 6 sensors on a shared I2C bus has been achieved. To complete this generation, combining all 6 sensors, with the buzzer and hysteresis logic, is next.

---

### Key Features  
**Validated (single sensor):**
- **2 Axes of Detection:** 6 VL53L8CX time of flight sensor readings for reliable distance measurement under varying noise and lighting conditions for 90° vertical and 135° horizontal per module.
- **Adaptive Ranging:** Frequency scales between **1 Hz (idle/far range)** and **6 Hz (near-range detection)** based on the closest detected object, with hysteresis (2 consecutive near readings to speed up; object gone 4+ seconds and 2 consecutive far readings to slow back down). Resolution held fixed at 4×4 to avoid the heavier reconfiguration cost of a resolution switch.
- **Smoothed Buzzer Alerts:** Proximity-based beep profiles that step one level at a time rather than jumping directly to a target profile.
- **Structured Power Optimization:** Configuration changes (ranging frequency, resolution, integration time, ranging mode, sensor power/sleep state, duty cycling, VHV recalibration interval) were tested one variable at a time (Tests 1–11) against a shared baseline, with current draw logged at each step to guide the final single-sensor configuration. This reduced average current draw from an estimated ~500 mA baseline to 86 mA — an ~83% reduction — on the single-sensor module.

**Validated (multi-sensor, in progress toward full 6-sensor array):**
- **Multi-Sensor I2C Addressing:** Multi-Sensor I2C Addressing: A hardware I2C multiplexer (TCA9548A/PCA9548A) successfully addresses all 6 VL53L8CX sensors simultaneously, each mounted at its intended final angle. Channel-to-angle mapping (each sensor's mountYawDeg/mountPitchDeg in firmware) has been verified against physical sensor position, not just assumed. An earlier unreliable version of multi-sensor addressing was root-caused to an intermittent short between adjacent breadboard pull-up resistor leads — not an addressing or firmware issue.Additionally, it is important to note that channel 4 on the MUX is not working and consistently causes channel 3 and onwards to fail. Thus, sensor 5 is being put on channel 5. A separate software-assigned addressing approach (sequenced LPn/shutdown pin control, no physical mux) was also attempted and has not yet been made reliable.
- **Per-Zone Sub-Angle Correction:** Each sensor's 4×4 zone grid is used to refine object direction beyond the sensor's overall mounting angle. The VL53L8CX's zone-to-real-world mapping was empirically determined (not assumed from documentation alone, which describes an internal lens flip but not how it interacts with this board's physical mounting) via three independent tests: a four-corner sweep, a column-order sweep, and a dead-center object test. All three confirmed the same mapping, giving validated 3D (x/y/z) lift-relative object positioning per sensor rather than a single flat distance value.
    - Currently two simplifications are made about how the current coordinate model is treated:
    1. All 6 sensors are assumed to be sharing one physical origin point. This does not account for their 1-2cm spread on the mounting plates.
    2. Yaw/pitch rotations are additively combined rather than using full rotation matrices. 
    - The current use-case of this product allows for these to be made. They will be reconsidered after real situational testing on the construction site.

**Client Collaboration:** Developed through Duke's EGR102 course; presented deliverables to Skanska and collaborated to meet client specifications - as part of **client focused development**. Also coordinated with a power consultant on different strategies to optimize the system's power consumption, aiming to reach industry grade efficiency and reliability.

---

### System Architecture
**Target has been validated:** VL53L8CX x6 (via I2C mux), ESP32-S3, Piezo Buzzer

---

### Technical Stack  
- **Hardware:** VL53L8CX (6 validated simultaneously), ESP32-S3-DevKitC-1 (WROOM-2), TCA9548A/PCA9548A I2C multiplexer (6-sensor operation validated), piezo buzzer, transistor driver  
- **Software:** Arduino IDE (C/C++), with ST's VL53L8CX ULD API, VSCode, Python + PyGame (single-sensor visualizer used during bring-up).
- **Documentation:** Structured power-characterization test tables (Tests 1–11).

---

### Code
- **1TOFSensorModule_PerformanceCode.ino -** Most up-to-date firmware for this generation. Single VL53L8CX sensor running adaptive 1/6 Hz ranging at fixed 4×4 resolution with hysteresis-based mode switching and smoothed buzzer transitions. Written as a validated single-sensor building block, with the architecture intended to scale to 6 sensors once testing with the MUX has been completed.
- **Test1_Default_Baseline.ino, Test2_FrequencySweep.ino, plus Test files 3-11, power_test_common_h.ino -** Isolated power-characterization test sketches, each varying a single parameter (ranging frequency, resolution, integration time, ranging mode, sensor power/sleep state, duty cycling, VHV repeat count) against a shared baseline defined in **power_test_common.h**. Used to build the current-draw comparison tables that informed the configuration in **1TOFSensorModule_PerformanceCode.ino**.
- **MUX_TestCode.ino -** TCA9548A/PCA9548A-style I2C channel multiplexing test code. Validated for reliable 6-sensor addressing across repeated resets; next step is polling each sensor to achieve distance data from individual sensors.
- **SoftwareassigningI2CSensors.ino -** In-progress alternate approach assigning individual I2C addresses to each VL53L8CX sensor via sequenced LPn pin control, instead of a physical multiplexer.

---

### Alternate Approaches Considered
Before settling on the static 6-sensor array, a mechanically-swept single-sensor design was also prototyped. This used a single VL53L7CX 8×8-zone sensor mounted on a 270° servo, sweeping the corner similarly in spirit to Generation 1's dual-ultrasonic servo approach. This exploration included trigonometric threshold scaling to correct for edge angles, self-masking to filter out the lift's own mounting structure, and an adaptive fast/slow sweep rate with hysteresis. It was set aside in favor of the static multi-sensor array that reduced power consumption and avoided the added mechanical complexity, wear, and single point of failure of a servo.Additionally, full simultaneous coverage from fixed sensors will promote accuracy and efficiency once the I2C address issues are resolved.

---

### Results  
- Single-sensor adaptive ranging and power-optimized configuration validated.
- Structured, single-variable power characterization (Tests 1–11) completed — reduced single-sensor average current draw ~83% (from an estimated ~500 mA baseline to 86 mA).
- 6-sensor I2C multiplexer addressing validated as stable across 5+ consecutive resets, after root-causing an earlier intermittent failure to a breadboard pull-up resistor short (not an addressing/firmware issue). IMPORTANT: channel 4 on mux causes consistent failing. For sensor 5 use channel 5 instead of channel 4 and for sensor 6 use channel 6.
- Full 6-sensor channel-to-angle mapping validated: running MultiSensorAngleCalibration.ino, placing an object close to one sensor at a time confirmed each channel's serial output matches its physical position — e.g. an object 100mm from the top-left sensor showed the Top-Left channel reporting ~100mm while all other channels reported no nearby object. This confirms the coordinate transform's angle table is correct, not just assumed.- I2C/Serial Re-Init Crash: Initializing all 6 sensors in a loop originally caused a Guru Meditation Error (null-pointer panic, PC: 0x00000000) partway through setup. Root cause: sensor_init() called Wire.begin() and Serial.begin() on every invocation (once per sensor), and repeatedly re-initializing the ESP32's I2C driver mid-program corrupted its internal state. Fixed by guarding both calls with a static bool flag in power_test_common.h so the I2C and Serial peripherals are only truly initialized once, on the first sensor, regardless of how many times sensor_init() is called.
- Zone-level angle correction validated via three independent physical tests on the Top-Middle sensor (corner sweep, column sweep, dead-center object), all consistent with each other. This confirmed that the sensor's true zone-to-direction mapping differs from the datasheet-derived assumption due to this board's physical mounting rotation.
- c

---

### Next Steps
1. Integrate the piezo buzzer and hysteresis logic with the multiplexed 6 sensors. 
2. Start planning and developing the PCB design to consolidate the product's footprint
3. Re-run power characterization on the full array with staggered polling.
4. Obtain system-level results (Coverage, blind-spot reduction, per-unit cost, ROI, etc.)
5. Explore possibly treating all 6 sensors as sharing one physical origin point (not accounting for their few-cm spread on the mount plate), and combining yaw/pitch rotations additively rather than with full rotation matrices. Both are considered acceptable for the current collision-warning use case and are deferred pending how later testing this semester goes.

---

### Team  
- **Project Leads:** Pierson Jones, Parker Jones, Jack Oakman, Aaryan Nanekar  
- **Client Partner:** Skanska USA

---

### License  
This project is currently **private and under client collaboration**. Portions may be shared publicly for academic or demonstration purposes only.

---

### Acknowledgements  
Special thanks to **Skanska, Duke University’s EGR102 Course, Dr. Glass, and Shane Trent** for mentorship, technical guidance, and access to testing and prototyping facilities.