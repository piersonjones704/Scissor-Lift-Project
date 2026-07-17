# CollisionAvoidance
# Scissor Lift Collision Detection Module

### Overview
This project, developed in collaboration with **Skanska**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.
Generation 1 used dual **ultrasonic sensors** on a single servo-swept module to provide proximity detection and adaptive audio alerts, improving situational awareness in dynamic construction environments. A single module was built and mounted on the **front-right corner of a Genie 1930ES scissor lift** as a proof of concept; full-perimeter coverage would require additional modules at each corner or side.

---

### Key Features
- **Dual Ultrasonic Detection:** Two HC-SR04 ultrasonic sensor readings, staggered on a shared servo sweep for reliable distance measurement under varying noise and lighting conditions — covering **60° vertical** and **220° horizontal** (25°–245°, centered at 135°) per module.
- **Lift-Relative Distance Correction:** Raw sensor readings are converted into true distance **from the lift body itself** — not just from the sensor — using per-sensor, per-angle trigonometric correction based on the Genie 1930ES's known dimensions (76cm × 187cm) and the sensor's mounting offset. Each of the two sensors covers a different side of the corner (short side vs. long side), with its own correction geometry, so alerts reflect actual clearance to the lift rather than raw line-of-sight distance.
- **Embedded Software:** Real-time Arduino firmware for continuous sensor polling, hysteresis-based active-sensor selection, adaptive non-blocking audio alerts, and signal filtering.
- **Robust Design:** Built for durability, scalability, and cost efficiency — **45% cheaper** than initial prototype iterations.
- **Data-Driven Development:** Design decisions guided by **Pugh matrices**, **scoring analyses**, and **peer-reviewed research**.
- **Client Collaboration:** Developed through Duke's EGR101 course; presented deliverables to Skanska and collaborated to meet client specifications as part of **client-focused development**.

---

### Technical Stack
- **Hardware:** HC-SR04 Ultrasonic Sensors, Arduino UNO R3, Arduino NANO, DC Active Piezo Buzzer 100dB, Transistor, Miuzei DC 270° Digital Servo
- **Software:** Arduino IDE (C/C++), VSCode, Processing (Java-visualization), Onshape & TinkerCAD (mechanical & electrical design)
- **Documentation:** Pugh screening matrices, design scoring tables, and technical memos written for client review

---

### Code
Firmware went through several iterations as sensor control and alerting logic were refined:

- **`USSensorV1.ino` / `SL_Prototype_2SensorNoMappingCode.ino`** — Earliest prototype. Two independent sensors, each with its own servo and speaker, sharing a single TRIG pin. Blocking beep logic with no serial output.
- **`SL_Prototype2SensorMappingCode.ino`** — Added a non-blocking beep state machine and serial telemetry (angle/distance data) to enable real-time visualization.
- **`2SensorUpdatedCode.c`** — Consolidated both sensors onto a single shared servo sweep, added closest-sensor hysteresis (requires 3 consecutive readings before switching active sensor), servo detach/reattach on proximity, battery voltage monitoring, and a fade-out beep tone.
- **`dual_sensor_radar_integrated.ino`** — Edge-detection logic validated on a **benchtop rig** (a table sized to approximate a section of the lift platform corner), paired with the `dual_sensor_radar_display.pde` Processing visualizer, before geometry correction was tied to the real lift.

**`TrigonometrySensingCode.ino`** — **Final firmware used for Generation 1.** Builds on the integrated dual-sensor sweep with **lift-relative trigonometric distance correction**: each sensor's raw reading is mapped to true distance from the Genie 1930ES's body based on servo angle and known lift dimensions, rather than raw sensor-to-object distance. Sweep range set to 25°–245° (220°), centered at 135°. Detection threshold set to 30cm from the lift body; hysteresis-based active-sensor selection and non-blocking adaptive beeper carried over from earlier iterations.

- **`dual_sensor_radar_display.pde`** — Processing visualization paired with the integrated firmware, rendering a live radar-style display of both sensors' readings and the detection boundary in real time during benchtop testing.

---

### Results
- Constructed a prototype of a single module and mounted it on the front-right corner of a Genie 1930ES scissor lift.
- Achieved **220° horizontal** (25°–245°) and **60° vertical** coverage from that module.
- Alerts are based on **corrected distance from the lift body** (accounting for sensor offset and lift geometry), not raw sensor distance, with a 30cm detection threshold.
- Prevents potential damages worth **$100–$10,000 per construction project**, with a per-module build cost of **~$40**.
- Demonstrated scalable, safety-first innovation with the possibility to provide a **>10,000% ROI** on equipment protection.

---

### Team
- **Project Leads:** Aaryan Nanekar, Veer Bhatia, Pierson Jones, Jack Oakman
- **Client Partner:** Skanska USA

---

### License
This project is currently **private and under client collaboration**. Portions may be shared publicly for academic or demonstration purposes only.

---

### Acknowledgements
Special thanks to **Skanska** and **Duke University's EGR101 Course** for technical guidance, mentorship, and access to prototyping facilities.
