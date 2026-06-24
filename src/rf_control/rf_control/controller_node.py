#!/usr/bin/env python3
import sys
import threading
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rf_msgs.msg import MoveCommand, Odometry as RfOdometry
from nav_msgs.msg import Odometry as RosOdometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


import os


class Go1RobotController(Node):
    def __init__(self) -> None:
        # Read robot number from environment variable, default to 1
        robot_num = os.environ.get("ROBOT_NUM", os.environ.get("ROBOT_NUMBER", "1"))
        node_namespace = f"R{robot_num}"
        robot_ns_default = f"go1_016{robot_num}"

        super().__init__("go1_robot_controller", namespace=node_namespace)

        # Declare parameters
        self.declare_parameter("robot_ns", robot_ns_default)
        self.declare_parameter("tolerance", 0.08)
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("max_linear_speed", 0.2)
        self.declare_parameter("max_angular_speed", 0.4)

        self.robot_ns = self.get_parameter("robot_ns").value
        self.tolerance = self.get_parameter("tolerance").value
        self.kp_linear = self.get_parameter("kp_linear").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value

        # Robot State variables
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.has_odom = False

        self.target_x = None
        self.target_y = None
        self.cmd_lock = threading.Lock()

        # State Machine States
        # 0: INIT, 1: STANDING_UP, 2: READY, 3: MOVING
        self.state = 0
        self.step_counter = 0

        # Create publishers and subscribers
        self.odom_pub = self.create_publisher(RfOdometry, "odom", 10)
        self.cmd_sub = self.create_subscription(
            MoveCommand, "move_cmd", self._on_move_cmd, 10
        )

        # Unitree Go1 specific topics
        # QoS profiles: best effort for cmd_vel and stand, reliable for odom
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Setup Go1 ROS interface
        odom_topic = f"/{self.robot_ns}/{self.robot_ns}/odom"
        cmd_vel_topic = f"/{self.robot_ns}/{self.robot_ns}/cmd_vel"
        stand_up_topic = f"/{self.robot_ns}/stand_up"
        stand_down_topic = f"/{self.robot_ns}/stand_down"

        self.go1_odom_sub = self.create_subscription(
            RosOdometry, odom_topic, self._on_go1_odom, reliable_qos
        )
        self.go1_cmd_vel_pub = self.create_publisher(
            Twist, cmd_vel_topic, best_effort_qos
        )
        self.go1_stand_up_pub = self.create_publisher(
            Empty, stand_up_topic, best_effort_qos
        )
        self.go1_stand_down_pub = self.create_publisher(
            Empty, stand_down_topic, best_effort_qos
        )

        # 20 Hz timer (50ms) for state machine and control loop
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"Go1RobotController initialized. Namespace: {self.robot_ns}"
        )

    def _on_go1_odom(self, msg: RosOdometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Quaternion orientation
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

        # Publish the state as rf_msgs/Odometry for local consumers
        rf_odom = RfOdometry()
        rf_odom.x = float(self.current_x)
        rf_odom.y = float(self.current_y)
        rf_odom.yaw = float(self.current_yaw)
        self.odom_pub.publish(rf_odom)

    def _on_move_cmd(self, msg: MoveCommand) -> None:
        if not self.has_odom:
            self.get_logger().warn(
                "Received move command but odometry is not available yet! Ignoring."
            )
            return

        with self.cmd_lock:
            # target_x and target_y are treated as relative offsets in the robot's local frame
            dx = msg.target_x
            dy = msg.target_y
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            self.target_x = self.current_x + dx * cos_yaw - dy * sin_yaw
            self.target_y = self.current_y + dx * sin_yaw + dy * cos_yaw
            self.get_logger().info(
                f"Received relative move command: ({dx:.2f}, {dy:.2f}). "
                f"Computed absolute target: ({self.target_x:.2f}, {self.target_y:.2f})"
            )

    def _control_loop(self) -> None:
        self.step_counter += 1

        if self.state == 0:  # INIT
            if self.step_counter >= 40:  # wait 2 seconds for discovery
                self.get_logger().info("Sending STAND UP command to Go1...")
                self.go1_stand_up_pub.publish(Empty())
                self.state = 1
                self.step_counter = 0

        elif self.state == 1:  # STANDING_UP
            if self.step_counter >= 100:  # wait 5 seconds standing up
                if not self.has_odom:
                    self.get_logger().warn("Waiting for Go1 odometry...")
                    return
                self.get_logger().info("Go1 is STANDING and READY.")
                self.state = 2

        elif self.state == 2:  # READY
            tx, ty = None, None
            with self.cmd_lock:
                if self.target_x is not None:
                    tx, ty = self.target_x, self.target_y

            if tx is not None:
                self.get_logger().info(f"Starting movement towards ({tx:.2f}, {ty:.2f})")
                self.state = 3

        elif self.state == 3:  # MOVING
            tx, ty = None, None
            with self.cmd_lock:
                if self.target_x is not None:
                    tx, ty = self.target_x, self.target_y

            if tx is None:
                self.state = 2
                return

            if not self.has_odom:
                self.get_logger().warn("Odometry lost while moving!")
                return

            dx = tx - self.current_x
            dy = ty - self.current_y
            distance = math.hypot(dx, dy)

            if distance < self.tolerance:
                self.get_logger().info(f"Target reached! Distance: {distance:.3f}m")
                # Stop the robot
                twist = Twist()
                self.go1_cmd_vel_pub.publish(twist)
                with self.cmd_lock:
                    self.target_x = None
                    self.target_y = None
                self.state = 2
            else:
                # Compute steering angle and drive velocities
                target_yaw = math.atan2(dy, dx)
                yaw_error = target_yaw - self.current_yaw
                
                # Normalize yaw error to [-pi, pi]
                yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

                twist = Twist()
                if abs(yaw_error) > 0.5:
                    # Face the target first
                    twist.angular.z = np.clip(
                        self.kp_angular * yaw_error, 
                        -self.max_angular_speed, 
                        self.max_angular_speed
                    )
                    twist.linear.x = 0.0
                else:
                    # Drive forward and adjust heading
                    twist.linear.x = np.clip(
                        self.kp_linear * distance, 
                        -self.max_linear_speed, 
                        self.max_linear_speed
                    )
                    twist.angular.z = np.clip(
                        self.kp_angular * yaw_error, 
                        -self.max_angular_speed, 
                        self.max_angular_speed
                    )
                self.go1_cmd_vel_pub.publish(twist)

    def shutdown(self) -> None:
        if rclpy.ok():
            self.get_logger().info("Shutting down. Sending STAND DOWN command...")
            # Stop velocity first
            twist = Twist()
            self.go1_cmd_vel_pub.publish(twist)
            self.go1_stand_down_pub.publish(Empty())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Go1RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
