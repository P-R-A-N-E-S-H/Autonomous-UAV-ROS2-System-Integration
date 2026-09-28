# 🎯 Member 2 Examiner Q&A Defense Master
**Role:** Member 2 — Simulation & Flight Stack Engineer  
**Focus:** Gazebo Harmonic, ArduPilot SITL, EKF3 Tuning, Aerodynamics, QGroundControl

---

### Q1: Why did you use Gazebo Harmonic (Gz-Sim 8) instead of older Gazebo Classic 11?
**Answer:**  
> "Gazebo Classic 11 reached End-of-Life (EOL) in early 2025. **Gazebo Harmonic (Gz-Sim 8)** is the current modern simulation standard supported in ROS 2 Jazzy. It offers:
> 1. **Modular Entity-Component System (ECS):** Substantially higher simulation performance and multi-threaded physics execution using the DART physics engine.
> 2. **Ogre 2.x Physically Based Rendering (PBR):** Realistic lighting, shadows, and camera lens distortions critical for synthetic training and testing of Vision-Language Models (VLM).
> 3. **Native `ros_gz_bridge` Support:** Zero-copy shared memory data exchange between simulator and ROS 2."

---

### Q2: How does ArduPilot SITL fuse Visual SLAM data when GPS is disabled?
**Answer:**  
> "ArduPilot runs a 24-state **Extended Kalman Filter (EKF3)**. By setting `EK3_SRC1_POSXY = 6` (External Navigation), the EKF fuses incoming `VISION_POSITION_ESTIMATE` MAVLink messages (originating from Member 4's Visual SLAM) with high-rate 250Hz IMU accelerometer and gyroscope data. The EKF dynamically estimates sensor bias, velocity states, and orientation corrections at 100Hz."

---

### Q3: Why did you set `COMPASS_USE = 0` in your indoor parameter file?
**Answer:**  
> "In indoor warehouses and industrial environments, magnetic field distortions caused by reinforced concrete, steel beams, and electrical conduits cause catastrophic compass yaw drift and 'toilet-bowling' oscillations. By disabling the compass and setting `EK3_SRC1_YAW = 6`, the drone's heading is derived purely from visual feature tracking and gyroscope integration."

---

### Q4: How are aerodynamic drag and motor response time constants modeled?
**Answer:**  
> "In `model.sdf`, we use Gazebo Harmonic's `gz::sim::systems::MulticopterMotorModel` with:
> - Motor Thrust: $T = k_m \cdot \omega^2$ ($k_m = 8.54858 \times 10^{-6}$)
> - Rotor Drag: $F_{drag} = C_{drag} \cdot \omega^2$ ($C_{drag} = 8.06428 \times 10^{-5}$)
> - Time Constants: $\tau_{up} = 12.5\text{ms}$ (spin-up acceleration) and $\tau_{down} = 25.0\text{ms}$ (spin-down deceleration).  
> This faithfully replicates physical motor rotor inertia and prevents unrealistic instantaneous torque generation."
