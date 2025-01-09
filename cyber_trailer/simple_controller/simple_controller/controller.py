import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cyber_interfaces.srv import Goal
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time


class SimpleProportionalController(Node):
    def __init__(self):
        """Initialize the Simple Proportional Controller Node."""
        super().__init__("simple_proportional_controller")

        ############## Control parameters ##############
        self.kp_linear_coarse = 0.8  # Coarse linear proportional gain
        self.kp_angular_coarse = 2.4  # Coarse angular proportional gain

        self.kp_linear_fine = 0.3  # Fine linear proportional gain
        self.kp_angular_fine = 1.0  # Fine angular proportional gain

        self.time_expected = 2.5  # Expected time to reach the goal

        # Initialize some variables
        self.goal_position = None  # Target position to reach
        self.current_position = None  # Current position of the robot
        self.current_orientation = None  # Current orientation of the robot
        self.new_goal = False  # Flag to indicate a new goal has been set
        self.cmdVel = Twist()  # Twist message for velocity commands
        self.goal_threshold = 0.5  # Threshold for goal acceptance

        # Create callback groups to manage concurrency
        cmd_cb_group = MutuallyExclusiveCallbackGroup()
        srv_cb_group = MutuallyExclusiveCallbackGroup()

        # Publisher for velocity commands
        self.cmdVelPub_ = self.create_publisher(
            Twist, "/cmd_vel", 10, callback_group=cmd_cb_group
        )

        # Subscription to odometry data
        self.odomSub_ = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=srv_cb_group,
        )

        # Service for receiving goal positions
        self.goal_service = self.create_service(Goal, "/goal", self.goal_callback)
        self.last_log_time = time.time()

    def odom_callback(self, msg):
        """Callback to handle odometry updates.

        Args:
            msg (Odometry): The odometry message containing the robot's pose.
        """
        # Store the current position and orientation from the odometry
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        # If a goal is set, calculate the control output
        if self.new_goal:
            self.proportional_control()

    def goal_callback(self, request, response):
        """Service callback to set a new goal position.

        Args:
            request (Goal.Request): The service request containing the goal pose.
            response (Goal.Response): The service response.

        Returns:
            Goal.Response: The response object, currently unused.
        """
        self.new_goal = True
        self.goal_position = (request.pose.position.x, request.pose.position.y)
        self.get_logger().info("Goal received: " + str(self.goal_position))
        response.success = True
        return response

    def proportional_control(self):
        """Calculate and publish velocity commands to reach the goal."""
        if not self.new_goal:
            return

        # Calculate the errors in position
        position_error_x = self.goal_position[0] - self.current_position[0]
        position_error_y = self.goal_position[1] - self.current_position[1]

        # Check if the robot has reached the goal
        if (
            abs(position_error_x) <= self.goal_threshold
            and abs(position_error_y) <= self.goal_threshold
        ):
            self.new_goal = False  # Goal reached, reset flag
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmdVelPub_.publish(cmd_vel)  # Stop the robot
            self.get_logger().info(
                "Goal reached, stopped at ("
                + str(self.current_position[0])
                + ", "
                + str(self.current_position[1])
                + ")"
            )
            self.get_logger().info(
                "Final errors: ("
                + str(position_error_x)
                + ", "
                + str(position_error_y)
                + ")"
            )
            return

        self.get_logger().warn(
            f"Pending errors: {position_error_x}, {position_error_y}",
            throttle_duration_sec=1.0,
        )

        # Compute the pending distance and linear velocity required to the goal
        distance = math.sqrt(position_error_x**2 + position_error_y**2)

        # Adjust linear velocity based on distance to goal
        if distance > self.goal_threshold * 6:
            linear_velocity = self.kp_linear_coarse * distance / self.time_expected
        else:
            linear_velocity = self.kp_linear_fine * distance / self.time_expected

        # Compute the desired angle and angular velocity
        desired_angle = math.atan2(position_error_y, position_error_x)
        current_angle = euler_from_quaternion(self.current_orientation)[2]
        angular_error = (desired_angle - current_angle + np.pi) % (2 * np.pi) - np.pi

        # Adjust angular velocity based on distance to goal
        if distance > self.goal_threshold * 6:
            angular_velocity = (
                self.kp_angular_coarse * angular_error / self.time_expected
            )
        else:
            angular_velocity = self.kp_angular_fine * angular_error / self.time_expected

        # Clip velocities to defined limits
        linear_velocity = np.clip(linear_velocity, -1.5, 1.5)
        angular_velocity = np.clip(angular_velocity, -0.7854, 0.7854)

        # Create the Twist message and publish the velocity command
        self.cmdVel.linear.x = linear_velocity
        self.cmdVel.angular.z = angular_velocity
        self.cmdVelPub_.publish(self.cmdVel)

def main(args=None):
    """Main entry point for the controller node."""
    rclpy.init(args=args)

    controller = SimpleProportionalController()
    exec = rclpy.executors.MultiThreadedExecutor(2)
    exec.add_node(controller)
    exec.spin()
    controller.get_logger().info("Exiting ...")
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt. Shutting down...")
    except Exception:
        print(f"Error occurred: {e}")
        # print(traceback.format_exc())
