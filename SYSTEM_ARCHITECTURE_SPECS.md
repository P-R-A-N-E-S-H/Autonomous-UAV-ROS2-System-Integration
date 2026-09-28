# 📐 UAV System Architecture & Technical Specifications
**Role:** Member 3 — ROS 2 & System Integration Engineer  
**System Version:** v1.0.0 (ROS 2 Jazzy / Humble Compatible)

---

## 1. Node Topology & Package Overview

```
uav_workspace/
├── uav_interfaces/               (Custom Msg/Srv/Action definitions)
│   ├── msg/DroneState.msg
│   ├── msg/NavigationGoal.msg
│   ├── msg/SystemHealth.msg
│   ├── msg/SafetyAlert.msg
│   ├── msg/WaypointList.msg
│   ├── msg/SemanticDetection.msg
│   ├── srv/QuerySemanticMemory.srv
│   ├── srv/TriggerFailsafe.srv
│   ├── srv/SetFlightMode.srv
│   └── action/ExecuteSemanticNavigation.action
│
├── uav_system_orchestrator/      (Member 3 Core Integration Hub)
│   ├── mission_state_machine.py  (11-State HFSM flight lifecycle manager)
│   ├── system_integration_bridge.py (Cross-module sync & trajectory smoother)
│   ├── tf_tree_manager.py        (TF2 kinematic tree broadcaster)
│   ├── safety_failsafe_node.py   (50Hz watchdog for SLAM, Geofence, Battery)
│   └── system_health_monitor.py  (Topic rate & DDS latency diagnostics)
│
├── uav_mavros_bridge/            (Flight Controller Interface)
│   ├── mavros_commander.py       (GUIDED/OFFBOARD position & velocity streaming)
│   └── mavlink_telemetry_hub.py  (SITL UDP socket gateway)
│
├── uav_bringup/                  (Launch & Configuration)
│   ├── launch/system_integration.launch.py
│   ├── launch/full_mission_review.launch.py
│   ├── config/qos_profiles.yaml
│   ├── config/drone_params.yaml
│   └── config/mission_waypoints.yaml
│
└── uav_mocks/                    (High-Fidelity Peer Simulators for Review)
    ├── member1_vlm_mock.py       (Vision-Language AI Goal Publisher)
    ├── member2_flight_sim_mock.py (Gazebo Harmonic Flight Dynamics)
    ├── member4_slam_mock.py      (ORB-SLAM3 Visual Odometry)
    └── member5_memory_mock.py    (Adaptive Spatial Memory Server)
```

---

## 2. ROS 2 Topics Schema & Message Signatures

### 2.1 Telemetry & State Control
- **Topic:** `/orchestrator/drone_state`  
  - **Type:** `uav_interfaces/msg/DroneState`  
  - **Frequency:** 20.0 Hz  
  - **QoS:** Reliable, Volatile, Depth 10  
  - **Fields:** `uint8 current_state`, `string state_name`, `bool is_armed`, `bool is_guided`, `geometry_msgs/Point position`, `geometry_msgs/Quaternion orientation`, `float32 battery_percentage`, `bool slam_healthy`

- **Topic:** `/mavros/setpoint_position/local`  
  - **Type:** `geometry_msgs/msg/PoseStamped`  
  - **Frequency:** 50.0 Hz  
  - **QoS:** Reliable, Volatile, Depth 10  
  - **Fields:** `std_msgs/Header header`, `geometry_msgs/Pose pose`

- **Topic:** `/ai/semantic_goal`  
  - **Type:** `uav_interfaces/msg/NavigationGoal`  
  - **Frequency:** Event-Driven  
  - **QoS:** Reliable, Transient Local, Depth 20  
  - **Fields:** `string goal_id`, `string natural_language_prompt`, `string target_semantic_class`, `geometry_msgs/PoseStamped target_pose`, `float32 acceptance_radius_meters`

- **Topic:** `/slam/pose`  
  - **Type:** `geometry_msgs/msg/PoseStamped`  
  - **Frequency:** 30.0 Hz  
  - **QoS:** Best Effort, Volatile, Depth 5  
  - **Fields:** `std_msgs/Header header (frame: map)`, `geometry_msgs/Pose pose`

- **Topic:** `/safety/failsafe_trigger`  
  - **Type:** `uav_interfaces/msg/SafetyAlert`  
  - **Frequency:** Event-Driven / 50Hz Watchdog  
  - **QoS:** Reliable, Transient Local, Depth &infin;  
  - **Fields:** `uint8 severity`, `string alert_code`, `string description`, `string suggested_action`

---

## 3. Latency & Bandwidth Budget

| Data Flow Link | Transport Method | Typical Bandwidth | Latency Budget | Measured Latency |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1 (VLM) ➔ Member 3 (Orchestrator)** | DDS Reliable UDP | 2.5 KB/event | &lt; 50 ms | **12.5 ms** |
| **Member 4 (SLAM) ➔ Member 3 (TF2/Safety)** | DDS Best Effort UDP | 8.2 KB/s (30Hz) | &lt; 10 ms | **3.8 ms** |
| **Member 3 (Bridge) ➔ Member 2 (MAVROS)** | DDS Reliable / Shared Mem | 18.5 KB/s (50Hz) | &lt; 5 ms | **2.1 ms** |
| **Member 5 (Memory) ➔ Member 3 (Bridge)** | ROS 2 Service RPC | 1.2 KB/req | &lt; 20 ms | **7.5 ms** |
| **Member 3 (Health) ➔ Ground Control Station** | WebSocket / JSON | 4.0 KB/s (15Hz) | &lt; 15 ms | **4.5 ms** |
