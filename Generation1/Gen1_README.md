# Scissor Lift Collision Prevention Module 
## Generation 1 - Sweeping Ultrasonic Sensors

### Overview  
This project, developed in collaboration with **Skanska**, aims to reduce jobsite accidents by integrating a **low-cost collision detection and avoidance system** into industrial scissor lifts.  

This module utilizes **dual ultrasonic sensors** on a single servo-swept module to provide proximity detection and adaptive audio alerts, improving situational awareness in dynamic construction environments. A single module was built and corner-mounted on the lift as a proof of concept; full-perimeter coverage would require additional modules at each corner or side.

---

### Key Features  
- **Dual Ultrasonic Detection:** Two HC-SR04 ultrasonic sensor readings, staggered on a shared servo sweep for reliable distance measurement under varying noise and lighting conditions — covering 60° vertical and 210° horizontal per module.
- **Embedded Software:** Real-time Arduino firmware for continuous sensor polling, hysteresis-based active-sensor selection, adaptive non-blocking audio alerts, and signal filtering.
- **Robust Design:** Built for durability, scalability, and cost efficiency — **45% cheaper** than initial prototype iterations.  
- **Data-Driven Development:** Design decisions guided by **Pugh matrices**, **scoring analyses**, and **peer-reviewed research**.  
- **Client Collaboration:** Developed through Duke's EGR101 course; presented deliverables to Skanska and collaborated to meet client specifications - as part of **client focused development**. 

---

### Technical Stack  
- **Hardware:** HC-SR04 Ultrasonic Sensors, Arduino UNO R3, Arduino NANO, DC Active Piezo Buzzer 100dB, Transistor, Miuzei DC 270° Digital Servo
- **Software:** Arduino IDE (C/C++), VSCode, Processing (Java-visualization), Onshape & TinkerCAD (mechanical & electrical design)
- **Documentation:** Pugh screening matrices, design scoring tables, and technical memos written for client review  

---

### Code
- **USSensorV1.ino / SL_Prototype_2SensorNoMappingCode.ino -**  Earliest prototype. Two independent sensors, each with its own servo and speaker, sharing a single TRIG pin. Blocking beep logic with no serial output.
- **SL_Prototype2SensorMappingCode.ino -** Added a non-blocking beep state machine and serial telemetry (angle/distance data) to enable real-time visualization.
- **2SensorUpdatedCode.c -** Consolidated both sensors onto a single shared servo sweep, added closest-sensor hysteresis (requires 3 consecutive readings before switching active sensor), servo detach/reattach on proximity, battery voltage monitoring, and a fade-out beep tone.
- **dual_sensor_radar_integrated.ino -** Final firmware used for this prototype generation. Single-servo staggered dual-sensor sweep (210° horizontal range) with hysteresis-based active-sensor selection and a non-blocking adaptive beeper. Detection logic was validated on a benchtop validation rig — a table sized to approximate a section of the lift platform corner — allowing repeatable edge-detection testing before the module was mounted on the actual lift.
- **dual_sensor_radar_display.pde -** Processing visualization paired with the final firmware, rendering a live radar-style display of both sensors' readings and the detection boundary in real time during benchtop testing.

---

### Results  
- Constructed a prototype of a single module and corner-mounted it on the lift.
- Achieved **210° horizontal** and **60° vertical** coverage from this module, validated on a benchtop rig before installation.
- As a full system with 9 modules, it prevents potential damages worth **$100-$10,000 per construction project** with a per-module build cost of **~$40**.  
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
Special thanks to **Skanska** and **Duke University’s EGR101 Course** for technical guidance, mentorship, and access to prototyping facilities.
