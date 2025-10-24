#!/usr/bin/env python3
# Copyright 2025 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")

        # Transient local QoS = "latched" behavior
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "robot_description", qos)

        # Declare and get the robot_description parameter
        self.declare_parameter("robot_description", "")
        description = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )

        if not description:
            self.get_logger().error("No robot_description parameter provided!")
            return

        # Publish once (latched)
        msg = String(data=description)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
