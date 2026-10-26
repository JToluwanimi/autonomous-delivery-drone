# Design and Development of an Autonomous Delivery Drone for Lightweight Packages

> **Status:** Designed / Prototyped / Tested
> **Author:** Toluwanimi Jesusina
> **Institution:** Bells University of Technology, Ota, Nigeria
> **Year:** 2024
> **Supervisor:** Dr. Adewuyi Philip A.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status](https://img.shields.io/badge/Status-Prototype-blue)]()

---

## 1. What is this?

A final-year Mechatronics Engineering capstone project focused on designing, prototyping, and testing a cost-effective autonomous delivery drone optimized for lightweight packages in urban environments. The system integrates a modular DJI F450 frame, Pixhawk autopilot, GPS waypoint navigation, ultrasonic obstacle detection, and a custom servo-actuated payload delivery mechanism.

**Note on terminology:** This project uses **GPS waypoint navigation** and **ultrasonic obstacle detection**. It does **not** implement SLAM, computer vision, or fully autonomous dynamic path planning. Those technologies were investigated during the literature review but were not implemented in the final prototype due to budget and time constraints. All limitations are documented in Section 9.

---

## 2. Why was it built?

Traditional last-mile delivery methods are inefficient for lightweight packages, facing challenges such as:

- **Urban congestion** — Vans and trucks struggle in dense environments.
- **High overhead** — Labor and vehicle maintenance costs outweigh the value of small packages.
- **Environmental impact** — Increased carbon emissions from inefficient routing.
- **Remote access** — Difficulty reaching areas with poor road infrastructure.

Existing commercial drones are optimized for photography, surveillance, or mapping — not for precise, affordable lightweight package delivery. This project addresses the need for a **specialized, low-cost UAV platform** capable of autonomous waypoint navigation and reliable payload deployment.

---

## 3. Objectives

| # | Objective | Status |
|---|-----------|--------|
| 1 | Design an autonomous delivery drone optimized for lightweight packages (< 2 kg) | ✅ Completed |
| 2 | Implement precise navigation and delivery systems | ✅ Completed |
| 3 | Ensure regulatory compliance and safety standards | ✅ Documented |
| 4 | Evaluate performance and feasibility in real-world scenarios | ✅ Tested |

---

## 4. System Architecture

### 4.1 High-Level Block Diagram

```
┌────────────────────────────────────────────────────────────────────────────┐
│                              INPUTS                                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   Ublox NEO- │  │  3DR Radio   │  │  HC-SR04     │  │   RC         │    │
│  │   M8N GPS    │  │  Telemetry   │  │  Ultrasonic  │  │   Transmitter│    │
│  │              │  │  (915 MHz)   │  │  Sensor      │  │              │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘    │
│         │                 │                 │                 │            │
└─────────┼─────────────────┼─────────────────┼─────────────────┼────────────┘
          │                 │                 │                 │
          ▼                 ▼                 ▼                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           PROCESSING                                        │
│                    ┌─────────────────────────┐                              │
│                    │   Pixhawk Autopilot     │                              │
│                    │   (PX4 / ArduPilot)     │                              │
│                    │                         │                              │
│                    │  ┌─────────────────┐    │                              │
│                    │  │ Extended Kalman │    │                              │
│                    │  │ Filter (EKF)    │    │                              │
│                    │  └─────────────────┘    │                              │
│                    └────────────┬────────────┘                              │
│                                 │                                           │
└─────────────────────────────────┼───────────────────────────────────────────┘
                                  │
          ┌───────────────────────┼───────────────────────┐
          │                       │                       │
          ▼                       ▼                       ▼
┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
│    CONTROL      │   │    CONTROL      │   │    CONTROL      │
│  4x ESCs        │   │  Servo Motor    │   │  Telemetry      │
│  (PWM Signal)   │   │  (Payload Door) │   │  (MAVLink)      │
└────────┬────────┘   └────────┬────────┘   └────────┬────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
│   ACTUATION     │   │   ACTUATION     │   │   FEEDBACK      │
│  4x BLDC        │   │  Payload Door   │   │  Ground Control │
│  A2212/13T      │   │  Open/Close     │   │  Station (GCS)  │
│  1000KV Motors  │   │                 │   │  QGroundControl │
└─────────────────┘   └─────────────────┘   └─────────────────┘
```

### 4.2 Signal Flow

| Signal | Source | Destination | Protocol |
|--------|--------|-------------|----------|
| GPS Position | Ublox NEO-M8N | Pixhawk | UART |
| Telemetry | Pixhawk | Ground Station | MAVLink (915 MHz) |
| RC Commands | Futaba Transmitter | Pixhawk | PWM |
| Motor Control | Pixhawk | ESCs | PWM (1000-2000 µs) |
| Obstacle Distance | HC-SR04 | Pixhawk | GPIO / I2C |
| Payload Release | Pixhawk | Servo | PWM |

---

## 5. Hardware Specifications

### 5.1 Bill of Materials

| Component | Model / Specification | Quantity | Purpose |
|-----------|----------------------|----------|---------|
| Frame | DJI F450 Flame Wheel | 1 | Structural chassis (X-frame) |
| Motors | BLDC A2212/13T 1000KV | 4 | Propulsion |
| ESCs | 30A Electronic Speed Controllers | 4 | Motor speed control |
| Propellers | 1045 (10" x 4.5") | 4 | Thrust generation |
| Battery | TCB 5200mAh 35C 14.8V LiPo | 1 | Power source |
| Autopilot | Pixhawk Flight Controller | 1 | Navigation & control |
| GPS | Ublox NEO-M8N | 1 | Position estimation |
| Telemetry | 3DR Radio (915 MHz) | 1 pair | Ground communication |
| Ultrasonic | HC-SR04 | 1 | Obstacle detection |
| Servo | SG90 / MG996R | 1 | Payload door actuation |
| Payload Box | Custom water-resistant 3D-printed | 1 | Package containment |
| Power Module | Pixhawk PM | 1 | Voltage/current sensing |
| RC System | Futaba T4YF (or equivalent) | 1 | Manual override |

### 5.2 Frame Geometry

- **Configuration:** X-frame quadcopter
- **Wheelbase:** 450 mm (diagonal)
- **Material:** Carbon fibre arms, aluminium alloy mounts
- **Weight (empty):** ~1.2 kg
- **Max Takeoff Weight (MTOW):** ~2.5 kg (including payload)
- **Payload Capacity:** Up to 2.0 kg (tested)

---

## 6. Software & Firmware

### 6.1 Flight Stack

| Layer | Software | Purpose |
|-------|----------|---------|
| Firmware | PX4 / ArduPilot | Flight control, sensor fusion, navigation |
| Ground Station | QGroundControl | Mission planning, telemetry monitoring |
| Communication | MAVLink | Telemetry protocol |
| Path Planning | Sector-based trajectory (S1-S5) | Smooth waypoint navigation |

### 6.2 Path Planning Sectors

The drone follows a defined trajectory from start to delivery point, divided into five sectors:

| Sector | Description |
|--------|-------------|
| S1 | Vertical ascent from launch point |
| S2 | Transition arc to cruise altitude |
| S3 | Horizontal cruise toward destination |
| S4 | Transition arc to descent |
| S5 | Vertical descent and landing |

This sector-based approach minimizes abrupt movements and ensures smooth transitions between flight phases.

### 6.3 Mission Execution Logic

```
1. Pre-flight: Calibrate ESCs, GPS lock, set home position
2. Takeoff: Ascend to cruise altitude (e.g., 30 m)
3. Navigation: Follow GPS waypoints (S1 → S2 → S3 → S4 → S5)
4. Obstacle Detection: Ultrasonic sensor monitors for obstacles
5. Delivery: At destination, servo opens payload door
6. Return: Execute reverse path to home
7. Landing: Descend and disarm
```

---

## 7. Engineering Design Decisions

| Decision | Rationale | Trade-off |
|----------|-----------|-----------|
| **X-frame vs. Hexacopter** | F450 X-frame is affordable, portable, and has sufficient thrust-to-weight for lightweight packages. | Less redundancy than hexacopter; not suitable for heavy payloads (> 3 kg). |
| **Ultrasonic vs. LiDAR** | Ultrasonic (HC-SR04) is low-cost and simple to interface. | Limited range (~4 m), susceptible to noise, no 3D mapping. |
| **Servo Door vs. Winch** | Servo door provides precise, reliable release without altering center of gravity mid-flight. | Requires careful waterproofing; single-point failure if servo jams. |
| **LiPo vs. Li-Ion** | LiPo offers high discharge rate and energy density for short, high-power flights. | Lower energy density than Li-Ion; limited flight time. |
| **GPS Waypoints vs. SLAM** | GPS waypoints are reliable for outdoor urban environments with clear sky view. | No indoor navigation; fails in GPS-denied environments. |

---

## 8. Implementation

### 8.1 Assembly Milestones

1. **Frame assembly** — Mounted F450 arms, power distribution board (PDB), and landing gear.
2. **Motor & ESC integration** — Soldered ESCs to PDB, connected motors, calibrated ESCs.
3. **Autopilot installation** — Mounted Pixhawk, connected GPS, telemetry, and RC receiver.
4. **Payload mechanism** — Installed servo-controlled door on custom 3D-printed box.
5. **Sensor integration** — Mounted HC-SR04 ultrasonic sensor facing downward/forward.
6. **Software configuration** — Flashed PX4, configured parameters in QGroundControl.
7. **Test flights** — Conducted incremental tests (hover → waypoint → payload).

### 8.2 Key Code / Configuration

- **ESC Calibration:** Used Arduino to send PWM signals (1000-2000 µs) to each ESC.
- **Mission Planning:** QGroundControl waypoint files (.plan) uploaded to Pixhawk.
- **Servo Trigger:** Configured Pixhawk AUX output to trigger servo at specific waypoint.

*[Note: Specific code files are available in the `/firmware/` and `/software/` directories.]*

---

## 9. Testing & Results

### 9.1 Flight Performance Data

| Test | Payload (kg) | Flight Time (min) | Energy Consumption (Wh) | Delivery Accuracy (m) |
|------|-------------|-------------------|-------------------------|----------------------|
| 1 | 0.5 | 25 | 200 | 1.5 |
| 2 | 1.0 | 20 | 220 | 1.2 |
| 3 | 1.5 | 18 | 240 | 1.8 |
| 4 | 2.0 | 15 | 260 | 1.0 |

**Observation:** Inverse relationship between payload and flight time. Heavier loads require more energy, reducing endurance.

![Flight Time vs Payload](media/performance_graphs/flight_time_vs_payload.png)

### 9.2 Energy Efficiency

| Payload (kg) | Distance (km) | Energy Consumption (Wh/km) |
|-------------|---------------|---------------------------|
| 0.5 | 4 | 33 |
| 1.0 | 4 | 44 |
| 1.5 | 4 | 53 |
| 2.0 | 4 | 65 |

![Energy Consumption vs Payload](media/performance_graphs/energy_consumption.png)

### 9.3 Obstacle Avoidance Performance

| Scenario | Success Rate (%) | Average Evasion Time (s) |
|----------|-----------------|-------------------------|
| Static Obstacles | 95 | 1.2 |
| Dynamic Obstacles (Slow) | 90 | 1.5 |
| Dynamic Obstacles (Fast) | 85 | 1.8 |

![Obstacle Avoidance](media/performance_graphs/obstacle_avoidance.png)

### 9.4 Delivery Accuracy Under Wind Conditions

| Wind Speed (m/s) | Deviation (m) | Accuracy (%) |
|-----------------|---------------|--------------|
| 0 | 0.5 | 99 |
| 0.5 | 0.7 | 98 |
| 1.0 | 0.9 | 95 |
| 1.5 | 1.0 | 90 |

![Delivery Accuracy vs Wind](media/performance_graphs/delivery_accuracy.png)

### 9.5 Thrust vs. Power Input

| Power Input (W) | Thrust at 70% Eff. (N) | Thrust at 80% Eff. (N) | Thrust at 90% Eff. (N) |
|-----------------|------------------------|------------------------|------------------------|
| 100 | 14 | 16 | 18 |
| 300 | 42 | 48 | 54 |
| 500 | 70 | 80 | 90 |
| 700 | 98 | 112 | 126 |
| 1000 | 140 | 160 | 180 |

![Thrust vs Power](media/performance_graphs/thrust_vs_power.png)

---

## 10. Limitations

### 10.1 Implemented vs. Explored

| Technology | Status |
|------------|--------|
| GPS Waypoint Navigation | ✅ Implemented |
| Ultrasonic Obstacle Detection | ✅ Implemented |
| Servo-Actuated Payload Door | ✅ Implemented |
| Pixhawk / PX4 Autopilot | ✅ Implemented |
| SLAM | ❌ Not Implemented (Literature Review Only) |
| Computer Vision | ❌ Not Implemented (Literature Review Only) |
| LiDAR | ❌ Not Implemented (Literature Review Only) |
| ArUco Marker Landing | ❌ Not Implemented (Literature Review Only) |

### 10.2 System Limitations

1. **Battery Technology** — Current LiPo limits flight time to ~25 minutes (no payload) and ~15 minutes (2 kg payload).
2. **Environmental Constraints** — Delivery accuracy degrades in wind speeds > 1.5 m/s.
3. **Obstacle Detection** — Ultrasonic sensor has limited range (~4 m) and cannot detect small or soft obstacles reliably.
4. **Regulatory** — BVLOS (Beyond Visual Line of Sight) operations are restricted in Nigeria by NCAA regulations.
5. **Payload Capacity** — Maximum tested payload is 2.0 kg; system is not suitable for heavier packages.
6. **No Indoor Navigation** — GPS-dependent; cannot operate in GPS-denied environments.

---

## 11. Future Development

| Priority | Enhancement | Expected Impact |
|----------|-------------|-----------------|
| 1 | Integrate Raspberry Pi + OpenMV for vision-based landing | Enable precision landing on markers |
| 2 | Upgrade to LiDAR / Time-of-Flight sensors | Robust obstacle detection |
| 3 | Implement SLAM for GPS-denied navigation | Indoor and urban canyon operation |
| 4 | Explore hydrogen fuel cells or solar integration | Extended flight time |
| 5 | Develop automated battery swapping station | Continuous operation |
| 6 | Collaborate with NCAA for BVLOS certification | Legal commercial deployment |

---

## 12. Lessons Learned

1. **Systems Integration is Non-Trivial** — Bridging the mechanical payload door with the software autopilot waypoint triggers required careful PWM signal mapping and failsafe configuration. A failure in any subsystem cascades to the whole.
2. **Data-Driven Design** — The inverse relationship between payload and flight time highlights the critical trade-offs required in UAV logistics. You cannot optimize for both without compromise.
3. **Safety is Paramount** — Rigorous pre-flight checklists, ESC calibration, and failsafe configuration are non-negotiable. A single oversight can lead to a flyaway or crash.
4. **Documentation Saves Time** — Keeping a detailed logbook of calibration values, test results, and configuration parameters saved hours of troubleshooting during flight tests.
5. **Budget Constraints Drive Innovation** — Using ultrasonic instead of LiDAR forced creative problem-solving and a deeper understanding of sensor limitations.

---

## 13. Media

### 13.1 Prototype Photos

*[Insert photos of assembled drone, payload mechanism, and flight tests]*

### 13.2 Diagrams

- Block Diagram (Figure 4.1)
- Circuit Diagram (Figure 4.2)
- Path Planning Sectors (Figure 2.2.1)

### 13.3 Videos

- [Hover Test Video](link)
- [Waypoint Navigation Test](link)
- [Payload Release Test](link)

### 13.4 Graphs

- Flight Time vs Payload
- Energy Consumption vs Payload
- Obstacle Avoidance Success Rate
- Delivery Accuracy vs Wind Speed
- Thrust vs Power Input

---

## 14. Repository Structure

```
autonomous-delivery-drone/
├── README.md
├── LICENSE
├── .gitignore
├── docs/
│   ├── Final_Year_Project_Report.pdf
│   ├── System_Architecture.md
│   ├── Design_Decisions.md
│   ├── Testing_and_Results.md
│   └── Regulatory_Considerations.md
├── hardware/
│   ├── BOM.csv
│   ├── schematics/
│   │   ├── block_diagram.png
│   │   └── circuit_diagram.png
│   └── datasheets/
├── firmware/
│   ├── esc_calibration/
│   └── payload_servo/
├── software/
│   ├── mission_plans/
│   └── data_analysis/
├── data/
│   ├── flight_time_vs_payload.csv
│   ├── energy_consumption.csv
│   ├── obstacle_avoidance.csv
│   └── delivery_accuracy.csv
└── media/
    ├── photos/
    ├── videos/
    └── performance_graphs/
```

---

## 15. References

1. Miranda, V. R., et al. (2022). Autonomous navigation system for a delivery drone. *Journal of Control, Automation and Electrical Systems*, 33, 141-155.
2. Aabid, A., et al. (2022). Reviews on design and development of unmanned aerial vehicle (UAV) for different applications. *J. Mech. Eng. Res. Dev*, 45(2), 53-69.
3. Saravanakumar, Y. N., et al. (2023). Power Sources for Unmanned Aerial Vehicles: A State-of-the-Art. *Applied Sciences*, 13(21), 11932.
4. Gupta, A., & Fernando, X. (2022). Simultaneous localization and mapping (SLAM) and data fusion in unmanned aerial vehicles. *Drones*, 6(4), 85.
5. NCAA. (2023). Statement of Compliance Part 21 Remotely Piloted Aircraft Systems (RPAS) Regulations.

---

## 16. License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## 17. Contact

**Toluwanimi Jesusina**
- Email: [jesusinatoluwanimi@gmail.com](mailto:jesusinatoluwanimi@gmail.com)
- LinkedIn: [linkedin.com/in/toluwanimi-jesusina](https://linkedin.com/in/your-profile)
```
