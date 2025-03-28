import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import time


class Create3Controller(Node):
    def __init__(self):
        super().__init__("create3_controller")
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/cmd_vel", QoSProfile(depth=10)
        )
        self.undock_client = ActionClient(self, Undock, "/undock")
        self.dock_client = ActionClient(self, Dock, "/dock")

    def undock(self):
        self.get_logger().info("Undocking...")
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Undock action server not available!")
            return

        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected!")
            return

        self.get_logger().info("Undock goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.is_docked:
            self.get_logger().info("Successfully undocked.")
        else:
                        self.get_logger().error("Failed to undock.")

    def dock(self):
        self.get_logger().info("Docking...")
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock action server not available!")
            return

        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected!")
            return

        self.get_logger().info("Dock goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.is_docked:
            self.get_logger().info("Successfully docked.")
        else:
            self.get_logger().error("Failed to dock.")

    def move_forward(self, distance, speed=0.2):
        self.get_logger().info(f"Moving forward {distance} meters...")
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

    def rotate(self, angle, angular_speed=0.5):
        self.get_logger().info(f"Rotating {angle} radians...")
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -angular_speed
        duration = abs(angle) / angular_speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Create3Controller()

    try:
        node.undock()
        node.move_forward(1.0)
        node.rotate(3.1415)  # Rotate 90 degrees
        node.move_forward(0.5)
        node.dock()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
