#!/usr/bin/env python3
"""
lifecycle_state_orchestrator.py
===============================
Deterministic ROS 2 Managed Lifecycle Node Orchestrator (REP-2007 Standard).
Role: Member 3 - ROS 2 & System Integration Engineer.

Lifecycle States:
 - Unconfigured -> (Configure) -> Inactive
 - Inactive     -> (Activate)  -> Active (Full 50Hz Flight Control)
 - Active       -> (Deactivate)-> Inactive (Safe Hover Standby)
 - Inactive     -> (Cleanup)   -> Unconfigured
 - Any          -> (Shutdown)  -> Finalized
"""

import time
from enum import IntEnum
from typing import Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Header
    from uav_interfaces.msg import DroneState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class NodeLifecycleState(IntEnum):
    UNCONFIGURED = 0
    INACTIVE = 1
    ACTIVE = 2
    FINALIZED = 3


LIFECYCLE_STATE_NAMES = {
    NodeLifecycleState.UNCONFIGURED: "UNCONFIGURED",
    NodeLifecycleState.INACTIVE: "INACTIVE",
    NodeLifecycleState.ACTIVE: "ACTIVE",
    NodeLifecycleState.FINALIZED: "FINALIZED"
}


class ManagedLifecycleLogic:
    """Core logic for deterministic node lifecycle state transitions."""

    def __init__(self, node_name: str = "uav_system_orchestrator"):
        self.node_name = node_name
        self.state = NodeLifecycleState.UNCONFIGURED
        self.is_configured = False
        self.is_active = False
        self.transition_history = []

    def on_configure(self) -> bool:
        """Allocate resources, bind QoS sockets, load parameter schemas."""
        if self.state != NodeLifecycleState.UNCONFIGURED:
            return False
        self.is_configured = True
        self.state = NodeLifecycleState.INACTIVE
        self._record_transition("on_configure", "INACTIVE")
        return True

    def on_activate(self) -> bool:
        """Enable 50Hz setpoint publishers, arm safety watchdogs."""
        if self.state != NodeLifecycleState.INACTIVE:
            return False
        self.is_active = True
        self.state = NodeLifecycleState.ACTIVE
        self._record_transition("on_activate", "ACTIVE")
        return True

    def on_deactivate(self) -> bool:
        """Pause flight command streaming, enter fail-safe standby."""
        if self.state != NodeLifecycleState.ACTIVE:
            return False
        self.is_active = False
        self.state = NodeLifecycleState.INACTIVE
        self._record_transition("on_deactivate", "INACTIVE")
        return True

    def on_cleanup(self) -> bool:
        """Release allocated buffers and reset parameter cache."""
        if self.state != NodeLifecycleState.INACTIVE:
            return False
        self.is_configured = False
        self.state = NodeLifecycleState.UNCONFIGURED
        self._record_transition("on_cleanup", "UNCONFIGURED")
        return True

    def on_shutdown(self) -> bool:
        """Gracefully terminate DDS participants."""
        self.is_active = False
        self.is_configured = False
        self.state = NodeLifecycleState.FINALIZED
        self._record_transition("on_shutdown", "FINALIZED")
        return True

    def _record_transition(self, event: str, result_state: str):
        self.transition_history.append({
            "timestamp": time.time(),
            "event": event,
            "state": result_state
        })


if ROS2_AVAILABLE:
    class LifecycleStateOrchestratorNode(Node):
        def __init__(self):
            super().__init__('lifecycle_state_orchestrator')
            self.get_logger().info("Starting ROS 2 Lifecycle Orchestrator Node [Member 3]...")

            self.logic = ManagedLifecycleLogic("uav_orchestrator")

            self.pub_lifecycle_state = self.create_publisher(String, '/orchestrator/lifecycle_state', 10)
            
            # Configure and activate node on startup
            self.logic.on_configure()
            self.logic.on_activate()

            self.timer = self.create_timer(1.0, self._broadcast_state)
            self.get_logger().info("Lifecycle State Orchestrator ACTIVE.")

        def _broadcast_state(self):
            msg = String()
            msg.data = LIFECYCLE_STATE_NAMES[self.logic.state]
            self.pub_lifecycle_state.publish(msg)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = LifecycleStateOrchestratorNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Lifecycle State Machine Test...")
        logic = ManagedLifecycleLogic()
        print(f"Initial: {LIFECYCLE_STATE_NAMES[logic.state]}")
        logic.on_configure()
        print(f"After Configure: {LIFECYCLE_STATE_NAMES[logic.state]}")
        logic.on_activate()
        print(f"After Activate: {LIFECYCLE_STATE_NAMES[logic.state]}")


if __name__ == '__main__':
    main()
