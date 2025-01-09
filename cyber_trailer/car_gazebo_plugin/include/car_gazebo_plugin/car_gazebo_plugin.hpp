#ifndef CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_
#define CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <cyber_interfaces/msg/cyber_drive_stamped.hpp>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Time.hh>
/**
 * @brief Utility function to determine the sign of a number.
 *
 * @param x The input number.
 * @return 1 if x > 0, -1 if x < 0, and 0 if x == 0.
 */
inline double sign_of(double x) {
    if (x > 0) {
        return 1;
    }
    if (x < 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief Class representing a bicycle model for vehicle dynamics.
 *
 * This class simulates the dynamics of a bicycle model, allowing for 
 * steering angle and curvature adjustments based on the wheelbase length.
 */
class BicycleModel {
    double wheelbase_length; ///< Length between front and rear wheels.
    double steer_angle; ///< Current steering angle.
    double max_steer_angle; ///< Maximum allowable steering angle.

public:
    /**
     * @brief Constructor for the BicycleModel class.
     *
     * @param wheelbase_length Length of the wheelbase.
     * @param steer_angle Initial steering angle (default is 0).
     * @param max_steer_angle Maximum steering angle (default is π/4).
     */
    BicycleModel(double wheelbase_length, double steer_angle = 0.0, double max_steer_angle = M_PI / 4.0) {
        this->wheelbase_length = wheelbase_length;
        this->steer_angle = steer_angle;
        this->max_steer_angle = max_steer_angle;
    }

    /**
     * @brief Set the steering angle, ensuring it does not exceed the maximum.
     *
     * @param steer_angle Desired steering angle.
     */
    void set_steer_angle(double steer_angle) {
        if (fabs(steer_angle) > this->max_steer_angle) {
            steer_angle = sign_of(steer_angle) * this->max_steer_angle;
        }
        this->steer_angle = steer_angle;
    }

    /**
     * @brief Set the rear curvature based on a curvature value.
     *
     * @param k_rear The desired rear curvature.
     */
    void set_rear_curvature(double k_rear) {
        this->steer_angle = atan(k_rear * this->wheelbase_length);
    }

    /**
     * @brief Get the current rear curvature.
     *
     * @return Current rear curvature.
     */
    double get_rear_curvature() {
        return tan(this->steer_angle) / this->wheelbase_length;
    }

    /**
     * @brief Set the front curvature based on a curvature value.
     *
     * @param k_front The desired front curvature.
     */
    void set_front_curvature(double k_front) {
        this->steer_angle = asin(this->wheelbase_length * k_front);
    }

    /**
     * @brief Get the current front curvature.
     *
     * @return Current front curvature.
     */
    double get_front_curvature() {
        return sin(this->steer_angle) / this->wheelbase_length;
    }

    /**
     * @brief Get the current steering angle.
     *
     * @return Current steering angle.
     */
    double get_steer_angle() { return this->steer_angle; }

    /**
     * @brief Create a new BicycleModel with an offset based on a lateral displacement.
     *
     * @param delta_y The lateral displacement.
     * @return A new BicycleModel instance with the adjusted parameters.
     */
    BicycleModel get_offset_bicycle(double delta_y) {
        if (this->steer_angle == 0.0) {
            return BicycleModel(wheelbase_length, 0.0);
        }
        BicycleModel new_bike(this->wheelbase_length);
        new_bike.set_rear_curvature(1. / (1. / this->get_rear_curvature() - delta_y));
        return new_bike;
    }
};

/**
 * @brief Class representing a steering model, inheriting from BicycleModel.
 *
 * This class simulates a steering system, allowing for left and right
 * bicycle models based on the front wheelbase width.
 */
class CyberModel : public BicycleModel {
    double front_wheelbase_width; ///< Width between the front wheels.

public:
    /**
     * @brief Constructor for the CyberModel class.
     *
     * @param wheelbase_length Length of the wheelbase.
     * @param front_wheelbase_width Width between the front wheels.
     * @param steer_angle Initial steering angle (default is 0).
     */
    CyberModel(double wheelbase_length, double front_wheelbase_width,
               double steer_angle = 0.0)
        : BicycleModel(wheelbase_length, steer_angle) {
        this->front_wheelbase_width = front_wheelbase_width;
    }

    /**
     * @brief Get the left bicycle model adjusted for steering.
     *
     * @return Adjusted BicycleModel for the left wheel.
     */
    BicycleModel get_left_bicycle() {
        return this->get_offset_bicycle(this->front_wheelbase_width / 2.);
    }

    /**
     * @brief Get the right bicycle model adjusted for steering.
     *
     * @return Adjusted BicycleModel for the right wheel.
     */
    BicycleModel get_right_bicycle() {
        return this->get_offset_bicycle(-1. * this->front_wheelbase_width / 2.);
    }
};

namespace car_gazebo_plugin {
    /**
     * @brief Class for the Car Gazebo plugin, integrating with the Gazebo simulation.
     *
     * This class controls the car model in the Gazebo environment, handling
     * physics interactions, ROS communication, and vehicle dynamics.
     */
    class CarGazeboPlugin : public gazebo::ModelPlugin {
    public:
        CarGazeboPlugin(); ///< Default constructor.

        /**
         * @brief Load the plugin and initialize parameters.
         *
         * @param model Pointer to the Gazebo model.
         * @param sdf Pointer to the SDF element containing configuration parameters.
         */
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:
        void Update(); ///< Update the plugin state in the simulation loop.

        using JointState = sensor_msgs::msg::JointState; ///< Alias for JointState message type.

        std::string robot_namespace_; ///< Namespace for the robot in ROS.

        gazebo::physics::ModelPtr model_; ///< Pointer to the Gazebo model.
        gazebo::physics::WorldPtr world_; ///< Pointer to the Gazebo world.

        gazebo::common::Time last_sim_time_; ///< Last simulation time.
        gazebo::common::Time last_update_time_; ///< Last update time for the plugin.
        double update_period_ms_; ///< Update period in milliseconds.

        gazebo::event::ConnectionPtr update_connection_; ///< Connection for simulation updates.

        // Joint configuration maps
        std::map<std::string, std::pair<gazebo::physics::JointPtr, gazebo::common::PID> > joints_;
        ///< Joint and PID pairs.
        std::map<std::string, double> joint_targets_; ///< Target positions for joints.

        rclcpp::Node::SharedPtr ros_node_; ///< ROS node for communication.
        rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_; ///< Publisher for joint states.
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; ///< Transform broadcaster for ROS.

        // Vehicle parameters
        double steer = 0; ///< Current steering value.
        double velocity = 0; ///< Current vehicle velocity.
        const double wheelbase_length = 0.62506; ///< Wheelbase length of the car.
        const double front_wheelbase_width = 0.48176; ///< Width of the front wheelbase.
        const double rear_wheelbase_width = 0.48013; ///< Width of the rear wheelbase.
        const double wheel_diameter = .06858; ///< Diameter of the wheels.
        const double max_speed = 3; ///< Maximum speed of the vehicle.
        const double max_turn_angle = M_PI / 4.0; ///< Maximum turning angle.
        CyberModel car_model = {0.62506, 0.48176};

        // Joint controller and parameters
        gazebo::physics::JointControllerPtr jc; ///< Joint controller for the model.
        gazebo::physics::JointPtr fl_str_joint; ///< Front left steering joint.
        gazebo::physics::JointPtr fr_str_joint; ///< Front right steering joint.
        gazebo::physics::JointPtr fl_axle_joint; ///< Front left axle joint.
        gazebo::physics::JointPtr fr_axle_joint; ///< Front right axle joint.
        gazebo::physics::JointPtr bl_axle_joint; ///< Back left axle joint.
        gazebo::physics::JointPtr br_axle_joint; ///< Back right axle joint.

        // PID controllers for joints
        gazebo::common::PID fl_pid, fr_pid, bl_pid, br_pid;

        // ROS subscriptions and publications
        rclcpp::Subscription<cyber_interfaces::msg::CyberDriveStamped>::SharedPtr cyberdrive_sub;
        ///< Subscription for drive commands.
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub; ///< Subscription for Twist commands.
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub; ///< Subscription for joystick input.
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odo_fl_pub; ///< Publisher for front left odometry.
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odo_fr_pub; ///< Publisher for front right odometry.
        rclcpp::Publisher<cyber_interfaces::msg::CyberDriveStamped>::SharedPtr cyberdrive_pub;
        ///< Publisher for CyberModel messages.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub; ///< Publisher for pose messages.
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub; ///< Publisher for odometry messages.

        /**
         * @brief Publish the current state of the vehicle.
         *
         * This function publishes the odometry information for both front axles.
         */
        void publish_state() {
            constexpr int ticks_per_revolution = 42; ///< Encoder ticks per revolution.

            std_msgs::msg::Int32 odo_fl; ///< Odometry message for front left wheel.
            odo_fl.data = static_cast<int>(fl_axle_joint->Position() * ticks_per_revolution / (M_2_PI));
            odo_fl_pub->publish(odo_fl); // Publish left odometry.

            std_msgs::msg::Int32 odo_fr; ///< Odometry message for front right wheel.
            odo_fr.data = static_cast<int>(fr_axle_joint->Position() * ticks_per_revolution / (M_2_PI));
            odo_fr_pub->publish(odo_fr); // Publish right odometry.
        }

        /**
         * @brief Callback function for receiving drive commands.
         *
         * @param msg Pointer to the received drive message.
         */
        void cyberdrive_callback(cyber_interfaces::msg::CyberDriveStamped::SharedPtr msg) {
            car_model.set_steer_angle(msg->drive.steering_angle);

            // Set the target positions for steering joints
            jc->SetPositionTarget(fl_str_joint->GetScopedName(), car_model.get_left_bicycle().get_steer_angle());
            jc->SetPositionTarget(fr_str_joint->GetScopedName(), -car_model.get_right_bicycle().get_steer_angle());

            // Calculate velocity based on curvature
            double curvature = car_model.get_rear_curvature();
            double speed = msg->drive.speed;
            double meters_per_rev = M_PI * wheel_diameter;
            double revs_per_sec = speed / meters_per_rev;
            double rads_per_sec = 2 * M_PI * revs_per_sec;

            if (curvature == 0) {
                jc->SetVelocityTarget(bl_axle_joint->GetScopedName(), rads_per_sec);
                jc->SetVelocityTarget(br_axle_joint->GetScopedName(), -rads_per_sec);
            } else {
                double radius = 1. / curvature;
                double left_radius = radius - rear_wheelbase_width / 2.;
                double right_radius = radius + rear_wheelbase_width / 2.;

                // Set the velocity targets for the left and right wheels
                jc->SetVelocityTarget(bl_axle_joint->GetScopedName(), rads_per_sec * left_radius / radius);
                jc->SetVelocityTarget(br_axle_joint->GetScopedName(), -rads_per_sec * right_radius / radius);
            }
        }

        /**
         * @brief Callback function for receiving Twist commands.
         *
         * @param msg Pointer to the received Twist message.
         */
        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
            cyber_interfaces::msg::CyberDriveStamped ad;
            ad.drive.speed = -msg->linear.x;

            // Adjust steering based on the linear and angular velocity.
            if (msg->linear.x == 0) {
                ad.drive.steering_angle = 0;
            } else {
                car_model.set_rear_curvature(msg->angular.z / msg->linear.x);
                ad.drive.steering_angle = -car_model.get_steer_angle();
            }
            cyberdrive_pub->publish(ad); // Publish command.
        }

    private:
        /**
         * @brief Helper function to retrieve a joint by its name.
         *
         * @param joint_name The name of the joint to retrieve.
         * @return Pointer to the joint, or null if not found.
         */
        gazebo::physics::JointPtr get_joint(const char *joint_name) const {
            auto joint = model_->GetJoint(joint_name);
            if (joint.get() == nullptr) {
                RCLCPP_ERROR(ros_node_->get_logger(), "Failed to get_joint %s", joint_name);
            }

            return joint;
        }
    };
} // namespace car_gazebo_plugin

#endif  // CAR_GAZEBO_PLUGIN__CAR_GAZEBO_PLUGIN_HPP_
