#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class LocationGoalSender(Node):
    def __init__(self):
        super().__init__("location_goal_sender")

        # (1) Find and load the YAML file once at startup
        lidar_nav_bringup_dir = get_package_share_directory("lidar_nav_bringup")
        landmarks_path = os.path.join(
            lidar_nav_bringup_dir, "map_landmarks", "landmarks_lab.yaml"
        )

        self.get_logger().info(f"Loading landmarks from: {landmarks_path}")

        with open(landmarks_path, "r") as f:
            data = yaml.safe_load(f) or {}

        # self.landmarks is just the dict under 'landmarks' in the YAML
        self.landmarks = data.get("landmarks", {})

        self.get_logger().info(f"Landmarks available: {list(self.landmarks.keys())}")

        # (2) Create Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # (3) Subscribe to matched_location (already lowercased & cleaned)
        self.sub = self.create_subscription(
            String,
            "matched_location",
            self.matched_location_callback,
            10,
        )

        self.get_logger().info("LocationGoalSender node ready.")

    def matched_location_callback(self, msg: String):
        """Called whenever a matched_location message arrives."""
        name = msg.data  # e.g. "trash can" or "door"

        if not name:
            self.get_logger().warn("Got empty matched_location, ignoring.")
            return

        self.get_logger().info(f"Got matched_location: '{name}'")

        # (4) Look up this name directly in the landmarks dict
        if name not in self.landmarks:
            self.get_logger().warn(f"No landmark found for '{name}'")
            return

        lm = self.landmarks[name]

        # Extract frame, position, orientation from YAML
        frame_id = lm.get("frame_id", "map")
        pos = lm["position"]
        ori = lm["orientation"]

        # (5) Build a PoseStamped for Nav2
        pose = PoseStamped()
        pose.header.frame_id = frame_id

        pose.pose.position.x = float(pos["x"])
        pose.pose.position.y = float(pos["y"])
        pose.pose.position.z = float(pos.get("z", 0.0))

        pose.pose.orientation.x = float(ori.get("x", 0.0))
        pose.pose.orientation.y = float(ori.get("y", 0.0))
        pose.pose.orientation.z = float(ori["z"])
        pose.pose.orientation.w = float(ori["w"])

        # (6) Build the NavigateToPose goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # (7) Wait for Nav2 action server and send goal
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 'navigate_to_pose' action server not available.")
            return

        self.get_logger().info(
            f"Sending goal for '{name}' to "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        # Fire-and-forget: we don't attach feedback / result callbacks here
        self.nav_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = LocationGoalSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
