# 🎙️ Member 2 Presentation Script — 10/10 Final Review
**Role:** Member 2 — Simulation & Flight Stack Engineer  
**Software/Tools:** Gazebo Harmonic, ArduPilot 4.5+ SITL, QGroundControl, ROS 2 Jazzy

---

## 📢 Spoken Presentation Script (Review Defense)

### 🟢 Part 1: Role Overview & Physics Simulation (0:00 - 1:30)
> *"Respected Examiners, I am **Member 2 (Simulation & Flight Stack Engineer)**.*  
> *My responsibility in this project is to build the **physics simulation world in Gazebo Harmonic**, model the **6-DOF quadrotor aerodynamics and sensors**, configure the **ArduPilot SITL autopilot**, and execute **autonomous waypoint survey missions**.*  
> *Rather than using a simplified toy simulator, I built a high-fidelity **1000Hz Gazebo Harmonic (Gz-Sim 8)** indoor warehouse environment with textured walls, realistic ambient lighting, and high-contrast landmarks to support Member 4's Visual SLAM and Member 1's Vision-Language Models."*

---

### 🟢 Part 2: Aerodynamics & Sensor Simulation (1:30 - 3:00)
> *"For the quadrotor dynamics, I parameterized the `iris_depth_camera` model with genuine rotor thrust curves:  
> $$T_i = k_m \cdot \omega_i^2 \quad (k_m = 8.55 \times 10^{-6} \text{ N}/(\text{rad/s})^2)$$  
> We simulate:  
> 1. An **RGB-D Depth Camera** tilted $15^\circ$ downward, streaming $640\times 480$ color and depth frames at 30Hz.  
> 2. A **250Hz IMU** with realistic accelerometer and gyroscope Gaussian white noise and bias drift.  
> 3. Motor time constants with $12.5\text{ms}$ rise time and $25\text{ms}$ decay to accurately model rotor inertia."*

---

### 🟢 Part 3: ArduPilot EKF3 GPS-Denied Tuning (3:00 - 4:15)
> *"A key technical highlight of my work is configuring **ArduPilot's 24-state EKF3 algorithm for GPS-denied flight**:  
> Since GPS signals cannot penetrate our indoor warehouse, I configured `EK3_SRC1_POSXY = 6` and `EK3_SRC1_YAW = 6`. This instructs the autopilot to fuse external visual odometry from Member 4's SLAM directly into its state estimation matrix.  
> Furthermore, I disabled the electronic compass (`COMPASS_USE = 0`) to eliminate severe magnetic interference from indoor steel beams."*

---

### 🟢 Part 4: Waypoint Missions & Live Demonstration (4:15 - 5:30)
> *"Using QGroundControl and our `flight_stack_mission_executor` node, we can inject complete multi-waypoint inspection plans:  
> 1. Autonomous Takeoff to 2.5m hover altitude.  
> 2. Inspection of the Medical Supply Crate at $(4.5, 3.2, 2.0)\text{m}$ with a 4-second dwell for AI object grounding.  
> 3. Inspection of the Hazardous Chemical Drum at $(-3.8, 6.1, 1.8)\text{m}$.  
> 4. Precision Return-To-Launch and landing.  
> All flight dynamics and mission executors have been verified with 100% test pass rate. I am ready for any technical questions."*
