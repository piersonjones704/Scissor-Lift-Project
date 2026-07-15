# Technical Memo: Design Criteria — Scissor Lift Collision Prevention System

**Team Name:** Scissor Lift Team 2  
**Members:** Pierson Jones, Jack Oakman, Veer Bhatia, Aaryan Nanekar

---

## Problem Statement
Scissor lifts are essential pieces of equipment on construction sites, but they present a significant safety risk due to limited visibility, blind spots, and operator distraction. Collisions with door frames, drywall, piping, and overhead systems on construction sites cause both financial damages and operator damages. Financial damages vary across incidents, with injuries/fatality settlements falling in the range of thousands to even millions of dollars depending on the severity [16]. Some indirect costs can be 2–10× as expensive as direct costs; these are unbudgeted, uninsured expenses that result from workplace accidents [17].  
Our design team aims to solve this problem by developing an object-detection device that alerts workers of potential collisions to reduce damage to site infrastructure while protecting workers from entrapment and falls.

---

## Importance of the Problem
The problem is significant for two reasons:

1. **Financial costs:** Construction companies are liable for repairs after scissor lift damage. Any collision, minor or major, can delay projects and cost thousands of dollars.
2. **Worker safety:** According to OSHA accident reports [12], scissor lifts are linked to dozens of deaths and hundreds of injuries annually, often due to collisions or entrapment. Operators risk pinning themselves between the lift and nearby structures.

---

## Limitations of the Status Quo
As discussed during the client interview, many operators improvise solutions (e.g., attaching pool noodles or padding to lifts). These minimize damage but **do not prevent collisions**. Existing technologies—object detection in cars [1][2], crane-lift collision avoidance systems [3], and proximity sensors in autonomous robots [3]—show the potential of sensor-based safety retrofits.  
However, scissor-lift environments have unique challenges: loud noise, variable operator training levels, strict OSHA and ANSI regulations [9][10], and the need for low-cost, durable solutions suitable for construction conditions. Many operators have 20+ years of experience and distrust complicated technology. Therefore, the solution must be **intuitive, universal, and fail-safe**, requiring minimal training and education.

---

## Users and Clients
- **Users:** Scissor-lift operators and site managers.  
  - Operators need an intuitive system; alerts and automatic stopping support this requirement.  
  - Managers value low maintenance.
- **Client:** Skanska USA Building Inc.  
  - Primary goal: reduce worker injuries and costs associated with injuries and site damage. They require a retrofit solution applicable to all standard scissor lifts.

*Shared goal:* finish projects quickly, safely, and at reduced cost.

---

## Key Function of the Solution
The system’s main function is to detect obstacles and warn users for objects within **~0.5 m** of the scissor lift and **automatically stop lift motion within ≤1 s** for objects within **~0.1 m** of the lift.

**Supporting functions:**
- Alert operators with audiovisual warnings.
- Withstand harsh construction environments (dust, vibration, weather).
- Require minimal maintenance and installation time.

---

## Table 1.1: Ranked Design Criteria

| **Rank** | **Objectives / Constraints** | **Target Value / Performance Criteria**                  | **Justification**                                                                 |
|:--:|---|---|---|
| **1** | Low Maintenance (objective) | ≤ 1 hour/month; automated startup diagnostics | Reduces false security from sensor failure [10]; aligns with OSHA findings on preventable accidents [12]. |
| **2** | Easy Setup (objective) | ≤ 30 minutes install; < 3 steps; only basic tools needed | Operators often lack advanced technical training [12]; minimizes downtime and increases adoption. |
| **3** | Durability (objective) | IP65+ rating; withstands vibration & 1 m drops; ≥ 3-year lifespan | Construction sites are harsh [7]; durability ensures long-term reliability and cost-effectiveness [18]. |
| **4** | Low Cost (constraint) | ≤ \$500 per lift (full system) | Cheaper than CAT retrofit cameras (\$600–\$800) [8]; avoids repair costs (\$5k–\$15k per incident). |
| **5** | Auto Stop (objective) | Detect up to ≥ 0.5 m; stop within ≤ 1 s for ~0.1 m | Scissor lifts move at 0.22–1.34 m/s; stopping within 1 s prevents collisions [4]. |

---

## Conclusion
These design criteria balance objectives and constraints, prioritizing maintenance, simplicity, durability, affordability, and automatic safety intervention. The criteria address both client and user needs, with a clear function that enhances scissor-lift safety. By preventing even a fraction of the hundreds of annual incidents involving scissor lifts [12], this device can save lives, reduce project delays, and offer a measurable ROI for construction companies.

---

## Appendix 1: Pairwise Comparison Chart for Design Criteria

|                     | Ease of Setup | Low Maintenance | Auto Stop | Durable | Low Cost | Total |
|---------------------|:-------------:|:---------------:|:---------:|:-------:|:--------:|:-----:|
| **Ease of Setup**   | ————          | 0               | 1         | 1       | 1        | **3** |
| **Low Maintenance** | 1             | ————            | 1         | 1       | 1        | **4** |
| **Auto Stop**       | 0             | 0               | ————      | 1       | 1        | **2** |
| **Durable**         | 0             | 0               | 0         | ————    | 1        | **1** |
| **Low Cost**        | 0             | 0               | 0         | 0       | ————     | **0** |

**Ranking (most → least important):**
1. Low Maintenance  
2. Ease of Setup  
3. Auto Stop  
4. Durable  
5. Low Cost
