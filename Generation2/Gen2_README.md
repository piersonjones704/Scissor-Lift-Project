# Scissor Lift Collision Prevention Module
## Generation 2 - Time of Flight Sensors

### Overview  
This project, developed in collaboration with **Skanska**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.  

Generation 2 is a work in progress. The goal is to replace Generation 1's single mechanically-swept ultrasonic module with a static array of  6 VL53L8CX time-of-flight sensors, each mounted at a different fixed angle around the lift — eliminating the need for a servo sweep entirely. 

This semester, single-sensor operation was validated end-to-end, including adaptive ranging and a power-optimized configuration. Reliably running all 6 sensors on a shared I2C bus has not yet been achieved and is the primary blocker to completing this generation.

---

### Key Features  
**Validated (single sensor):**
- **2 Axes of Detection:** 6 VL53L8CX time of flight sensor readings for reliable distance measurement under varying noise and lighting conditions for 90° vertical and 135° horizontal per module.
- **Adaptive Ranging:** Frequency scales between **1 Hz (idle/far range)** and **6 Hz (near-range detection)** based on the closest detected object, with hysteresis (2 consecutive near readings to speed up; object gone 4+ seconds and 2 consecutive far readings to slow back down). Resolution held fixed at 4×4 to avoid the heavier reconfiguration cost of a resolution switch.
- **Smoothed Buzzer Alerts:** Proximity-based beep profiles that step one level at a time rather than jumping directly to a target profile.
- **Structured Power Optimization:** Configuration changes (ranging frequency, resolution, integration time, ranging mode, sensor power/sleep state, duty cycling, VHV recalibration interval) were tested one variable at a time (Tests 1–11) against a shared baseline, with current draw logged at each step to guide the final single-sensor configuration. This reduced average current draw from an estimated ~500 mA baseline to 86 mA — an ~83% reduction — on the single-sensor module.

**In progress (full 6-sensor array):**
- **Multi-Sensor I2C Addressing:** Two approaches have been attempted so far — a hardware I2C multiplexer (TCA9548A/PCA9548A-style channel switching) and software-assigned individual addresses via sequenced LPn/shutdown pin control. Neither has been reliable with more than one sensor active.

**Client Collaboration:** Developed through Duke's EGR102 course; presented deliverables to Skanska and collaborated to meet client specifications - as part of **client focused development**. Also coordinated with a power consultant on different strategies to optimize the system's power consumption, aiming to reach industry grade efficiency and reliability.

---

### System Architecture
**Currently validated:** VL53L8CX x1, ESP32-S3, Piezo Buzzer

**Target:** VL53L8CX x6, I2C Addressing (mux or software assigned), ESP32-S3, Piezo Buzzer

---

### Technical Stack  
- **Hardware:** VL53L8CX (1 validated, 6 targeted), ESP32-S3-DevKitC-1 (WROOM-2), TCA9548A/PCA9548A I2C multiplexer (in-progress, not yet reliable), piezo buzzer, transistor driver  
- **Software:** Arduino IDE (C/C++), with ST's VL53L8CX ULD API, VSCode, Python + PyGame (single-sensor visualizer used during bring-up).
- **Documentation:** Structured power-characterization test tables (Tests 1–11).

---

### Code
- **1TOFSensorModule_PerformanceCode.ino -** Most up-to-date firmware for this generation. Single VL53L8CX sensor running adaptive 1/6 Hz ranging at fixed 4×4 resolution with hysteresis-based mode switching and smoothed buzzer transitions. Written as a validated single-sensor building block, with the architecture intended to scale to 6 sensors once addressing is resolved.
- **Test1_Default_Baseline.ino, Test2_FrequencySweep.ino, plus Test files 3-11, power_test_common_h.ino -** Isolated power-characterization test sketches, each varying a single parameter (ranging frequency, resolution, integration time, ranging mode, sensor power/sleep state, duty cycling, VHV repeat count) against a shared baseline defined in **power_test_common.h**. Used to build the current-draw comparison tables that informed the configuration in **1TOFSensorModule_PerformanceCode.ino**.
- **MUX_TestCode.ino -** In-progress work testing TCA9548A/PCA9548A-style I2C channel multiplexing to support multiple sensors on one bus.
- **SoftwareassigningI2CSensors.ino -** In-progress alternate approach assigning individual I2C addresses to each VL53L8CX sensor via sequenced LPn pin control, instead of a physical multiplexer.

---

### Alternate Approaches Considered
Before settling on the static 6-sensor array, a mechanically-swept single-sensor design was also prototyped. This used a single VL53L7CX 8×8-zone sensor mounted on a 270° servo, sweeping the corner similarly in spirit to Generation 1's dual-ultrasonic servo approach. This exploration included trigonometric threshold scaling to correct for edge angles, self-masking to filter out the lift's own mounting structure, and an adaptive fast/slow sweep rate with hysteresis. It was set aside in favor of the static multi-sensor array that reduced power consumption and avoided the added mechanical complexity, wear, and single point of failure of a servo.Additionally, full simultaneous coverage from fixed sensors will promote accuracy and efficiency once the I2C address issues are resolved.

---

### Results  
- Single-sensor adaptive ranging and power-optimized configuration validated.
- Structured, single-variable power characterization (Tests 1–11) completed — reduced single-sensor average current draw ~83% (from an estimated ~500 mA baseline to 86 mA).

---

### Next Steps
1. Resolve I2C addressing for 6 sensors (multiplexer or software address assignment).
2. Mount sensors at final angles and validate combined coverage.
3. Re-run power characterization on the full array with staggered polling.
4. Obtain system-level results (Coverage, blind-spot reduction, per-unit cost, ROI, etc.)

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