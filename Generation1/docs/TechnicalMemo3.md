# Technical Memo: Testing Plan – Scissor Lift Collision Prevention System

**Team Name:** Scissor Lift Team 2  
**Members:** Pierson Jones, Jack Oakman, Veer Bhatia, Aaryan Nanekar

---

## Design Criteria and Function

Scissor lifts can collide with many objects around a given construction site (e.g., door frames, piping, columns). These collisions result in heavy damage, financial burdens, and sometimes injuries or fatalities.  
An object-detection device is necessary to alert workers to potential collisions, thereby reducing overall damage and injuries on the construction site.

With this concept in mind, we ranked our design criteria in order of importance: **low maintenance** was the top priority, followed by **ease of setup**, **durability**, and **affordability**.  
To achieve these goals, we decomposed our product into key components — **sensor technology**, **mounting**, **alerting system**, and **casing**.

While prototyping, our most successful configuration used a **LiDAR Time-of-Flight sensor mounted on a servo motor**. We also tested **ultrasonic sensors** by placing them back-to-back and allowing them to rotate simultaneously.

---

## Role of Testing

Testing our sensor module prototype is essential to verify that it correctly detects objects while mounted on the lift.  
We must confirm that the **detection distance** and **angular coverage** meet our specifications — approximately **50 cm range** and **15° field of view (FOV)**.

Below, we outline a series of quantitative tests that evaluate the prototype’s ability to meet these performance criteria.

---

## Test 1 – Maximum Sensor Detection Distance

1. Connect an Arduino and ultrasonic sensor module to a speaker that beeps upon detection.  
   Place an object **5 cm** from the sensor, power the Arduino, and verify the beep.  
   Move the object back in **5 cm increments** until **50 cm** is reached.
2. Measure the distance using a **tape measure**.
3. Repeat the test at each increment until 50 cm is reached (10 trials).  
   Detection beyond 50 cm is not required per design specification.
4. Conducted entirely by the **design team**; client involvement not required.

---

## Test 2 – Maximum Sensor Field of View (FOV)

1. Using the same Arduino setup, position the object **3°** off-axis from the sensor’s center line.  
   Power the Arduino and verify the beep. Increase the angle by **3° increments** until **15°**.  
   Repeat on the opposite side.
2. Measure angle using a **protractor**.
3. Repeat 5 times per side (at 3° intervals).  
   The FOV requirement is ±15°; wider detection is unnecessary since the sensor will rotate on a servo motor.
4. Conducted by the **design team**; client involvement not required.

> **Note:** For both Test 1 and Test 2, if any **false positives or false negatives** occur, the test will not be considered passed.

---

## Test 3 – Mount Weight Capacity

1. Attach the mount **perpendicular to the floor** (parallel to table legs).  
   Incrementally hang weights (5 g steps) using a light but strong cord until the mount slips.
2. Measure all masses with a **scale**, summing total weight at failure.
3. Repeat the procedure **five times** for accuracy and consistency.
4. Conducted by the **design team**; client involvement not required.

---

## Test 4 – Mount and Casing Drop Durability

1. Place the assembled sensor module (without circuitry) inside its 3D-printed housing and attach it to its magnetic mount.  
   Drop the assembly from **0.5 m to 2.5 m** in **0.5 m increments** onto a hard, flat surface to simulate accidental drops.  
   After each drop, inspect for cracks, deformation, or detachment.
2. Rate damage using the **1–5 User-Defined Scale** (see Appendix Table 1).
3. Repeat each height test **three times** unless the module sustains major damage earlier.
4. Conducted by the **design team**; client involvement not required.

---

## Appendix

### Table 1 – User-Defined Scale: Damage After Drop

| **Ranking** | **Criteria: Damage After the Drop** |
|--------------|-------------------------------------|
| **1** | Major structural damage: cracking, delamination, or complete break of mount or case; sensor/servo detached or nonfunctional. |
| **2** | Visible cracking or permanent deformation compromising alignment or protection; mechanical integrity lost but parts still partially attached. |
| **3** | Minor cracks or cosmetic damage; sensor alignment slightly shifted (≤ 5°) but system still operational after repositioning. |
| **4** | Small scuffs, abrasions, or dents only; no cracks; sensor alignment remains within tolerance; full function retained. |
| **5** | No visible or functional damage; alignment unchanged; all components firmly secured; appears and operates as new. |

---
