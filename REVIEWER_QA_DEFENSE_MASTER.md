# 🎯 Examiner Q&A Defense Master — 25+ Tough Review Questions
**Role:** Member 3 — ROS 2 & System Integration Engineer  
**Objective:** Provide rock-solid, mathematically rigorous, and impressive answers to every question an examiner can ask.

---

## 🏛️ CATEGORY 1: ROS 2 Architecture & Middleware

### Q1: Why did your team use ROS 2 Jazzy instead of ROS 1 Noetic?
**Answer:**  
> "ROS 1 relies on a centralized `roscore` master, which creates a single point of failure (SPOF)—if `roscore` crashes, the entire flight vehicle is lost. In contrast, **ROS 2 Jazzy utilizes the OMG-standard DDS (Data Distribution Service)** middleware:
> 1. **Decentralized Discovery:** Dynamic participant discovery over UDP multicast with no central broker.
> 2. **Configurable QoS:** Granular control over Reliability, Durability, Deadlines, and Liveliness per topic.
> 3. **Deterministic Real-Time:** Support for intra-process zero-copy communication and real-time executors.
> 4. **Modern Support:** Long-Term Support (LTS) release maintained with modern Python 3.12+ and C++20."

---

### Q2: How does ROS 2 DDS manage intra-process vs inter-process communication?
**Answer:**  
> "When multiple ROS 2 nodes (e.g., our state machine and trajectory bridge) run within the same process container (composable nodes), ROS 2 can bypass DDS network serialization entirely and pass memory pointers using `std::unique_ptr` (zero-copy transport). For inter-process communication between AI perception and MAVROS, DDS uses shared memory (`CycloneDDS` / `FastDDS` loan mechanism) or loopback UDP sockets, achieving sub-millisecond latencies."

---

## 🧭 CATEGORY 2: Coordinate Transformations (TF2) & Kinematics

### Q3: How do you handle coordinate drift between the flight controller odometry (`odom`) and the Visual SLAM map (`map`)?
**Answer:**  
> "The flight controller's internal EKF estimates high-rate (100Hz) odometry in the `odom` frame, which has low latency but suffers from unbounded drift over time. Member 4's Visual SLAM provides loop-closure corrected absolute poses in the `map` frame at 30Hz.  
> As Member 3, I maintain the transformation:
> $$\mathbf{T}_{map}^{odom} = \mathbf{T}_{map}^{base\_link} \cdot (\mathbf{T}_{odom}^{base\_link})^{-1}$$
> When a loop closure occurs in SLAM, only the transform $\mathbf{T}_{map}^{odom}$ is updated in the `tf2` buffer. This guarantees that high-frequency control setpoints sent to the flight controller remain mathematically smooth without inducing sudden physical jerks on the drone motors."

---

### Q4: Why use Quaternions instead of Euler angles for orientation?
**Answer:**  
> "Euler angles $(\phi, \theta, \psi)$ suffer from **gimbal lock** at $\theta = \pm 90^\circ$ (singularity where pitch causes roll and yaw axes to align, losing one degree of freedom). Quaternions $\mathbf{q} = q_w + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}$ operate in four-dimensional space $\mathbb{S}^3$ and provide:
> 1. Singularity-free 3D rotation representation.
> 2. Smooth spherical linear interpolation (SLERP) for waypoint orientation.
> 3. Computationally efficient concatenation via quaternion multiplication."

---

## 🚁 CATEGORY 3: MAVROS & Flight Stack Control

### Q5: What flight mode does your MAVROS bridge use to command autonomous waypoints?
**Answer:**  
> "In ArduPilot SITL, we switch to **`GUIDED` mode** (or `OFFBOARD` mode in PX4). In `GUIDED` mode, the flight controller accepts external 3D position and velocity setpoints via MAVROS over `/mavros/setpoint_position/local`. We stream setpoints continuously at **50Hz**; if the stream drops below 2Hz, the autopilot's onboard safety watchdog automatically triggers a fallback to `ALT_HOLD` or `LOITER`."

---

### Q6: How do you prevent abrupt acceleration spikes when receiving a new AI goal?
**Answer:**  
> "We implemented a 2nd-order **Trajectory Smoother** in `system_integration_bridge.py`. Rather than passing step-function coordinates directly to MAVROS, the smoother limits the maximum velocity to $1.8\text{ m/s}$ and maximum acceleration to $1.2\text{ m/s}^2$ using a discrete-time velocity damping curve:
> $$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \text{clamp}(K_p \mathbf{e}(t) - \mathbf{v}(t), -\mathbf{a}_{max}\Delta t, +\mathbf{a}_{max}\Delta t)$$
> This guarantees smooth, stable flight without actuator saturation."

---

## 🛡️ CATEGORY 4: Safety, Fail-Safes & Reliability

### Q7: What happens if Visual SLAM tracking is lost in a dark or smoke-filled room?
**Answer:**  
> "Our `safety_failsafe_node` runs a 50Hz watchdog. If no pose update is received from Member 4 for $\Delta t > 350\text{ms}$:
> 1. The watchdog publishes a `SafetyAlert` with code `SLAM_TRACKING_LOST`.
> 2. The state machine shifts from `SEMANTIC_GOAL_NAV` to `EMERGENCY_HOLD`.
> 3. The flight controller is commanded to hold current altitude and zero out horizontal velocity using optical flow / IMU inertial integration.
> 4. If tracking is recovered within 3.0 seconds, the mission resumes. Otherwise, the drone initiates a gentle vertical touchdown at $0.3\text{ m/s}$."

---

### Q8: How is the geofence implemented and what happens upon breach?
**Answer:**  
> "We define a 3D cylindrical bounding volume: horizontal radial limit $R_{xy} = \sqrt{x^2 + y^2} \le 25.0\text{m}$ and vertical envelope $0.2\text{m} \le z \le 12.0\text{m}$.  
> If either boundary is breached, the safety supervisor triggers an automatic override to `RETURN_TO_HOME (RTH)`, commanding the drone to ascend to safety height and backtrack along the recorded keyframe path to the launch coordinates."

---

### Q9: How did you test and validate your integration code?
**Answer:**  
> "I developed a comprehensive automated test suite in `tests/` with **16 unit and integration tests** covering:
> 1. All 11 state machine transitions and command payloads.
> 2. DDS latency and packet drop calculations under bandwidth constraints.
> 3. Quaternion algebra and kinematic frame tree validity.
> 4. Watchdog response times under simulated SLAM loss and battery drain.  
> All 16 tests pass with 100% success rate in $0.30$ seconds."
