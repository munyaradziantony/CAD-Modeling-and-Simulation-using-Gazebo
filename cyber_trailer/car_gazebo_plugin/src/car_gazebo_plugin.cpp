#include "car_gazebo_plugin/car_gazebo_plugin.hpp"


namespace car_gazebo_plugin {
    /**
     * @brief Class representing a car plugin for Gazebo.
     *
     * This class implements a Gazebo model plugin for a car. It handles loading the
     * model, configuring the physics, and subscribing to ROS topics for commands
     * and publishing state information.
     */
    CarGazeboPlugin::CarGazeboPlugin(): robot_namespace_{""}, ///< Namespace for the robot
                                        last_sim_time_{0}, ///< Last simulation time
                                        last_update_time_{0}, ///< Last update time for the plugin
                                        update_period_ms_{8} {
        ///< Update period in milliseconds
    }

    /**
     * @brief Load the plugin and initialize parameters.
     *
     * This method is called when the plugin is loaded. It initializes the model,
     * sets up ROS communications, and configures the joint controllers.
     *
     * @param model Pointer to the Gazebo model.
     * @param sdf Pointer to the SDF element containing configuration parameters.
     */
    void CarGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
        // Get model and world references
        model_ = model;
        world_ = model_->GetWorld();
        auto physicsEngine = world_->Physics();
        physicsEngine->SetParam("friction_model", std::string{"cone_model"}); // Set friction model

        // Load the robot namespace from SDF
        if (sdf->HasElement("robotNamespace")) {
            robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        }

        // Set up ROS node, publisher, and subscriber
        ros_node_ = gazebo_ros::Node::Get(sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "Loading Car Gazebo Plugin");

        // Publisher for joint states
        joint_state_pub_ = ros_node_->create_publisher<JointState>("/joint_states", rclcpp::SensorDataQoS());

        // Find and configure joints
        auto allJoints = model_->GetJoints();
        RCLCPP_DEBUG(ros_node_->get_logger(), "All joints:");
        for (auto const &j: allJoints) {
            RCLCPP_DEBUG(ros_node_->get_logger(), j->GetName().c_str()); // Log all joint names
        }

        // Initialize joint controllers
        for (auto const &j: allJoints) {
            if (j->GetType() == gazebo::physics::Joint::FIXED_JOINT) {
                continue; // Skip fixed joints
            }

            // Initialize PID controllers for the joints
            auto pid = gazebo::common::PID{};
            pid.SetPGain(200.0); // Proportional gain
            pid.SetIGain(0.0); // Integral gain
            pid.SetDGain(0.0); // Derivative gain

            auto const &name = j->GetName();
            joints_[name] = std::make_pair(j, pid); // Store joint and its PID controller
            joint_targets_[name] = 0.0; // Initialize target position
        }

        RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");
        for (auto const &j: joints_) {
            RCLCPP_DEBUG(ros_node_->get_logger(), j.first.c_str()); // Log configured joints
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*ros_node_);

        // Setup joint controller parameters
        {
            RCLCPP_INFO(ros_node_->get_logger(), "Connected to model %s", model_->GetName().c_str());

            jc = model_->GetJointController(); // Get the joint controller

            // Configure PID for front left and right steering joints
            fl_pid = gazebo::common::PID(1, 0, 0);
            fl_str_joint = get_joint("chassis_fla_joint");
            jc->SetPositionPID(fl_str_joint->GetScopedName(), fl_pid);

            fr_pid = gazebo::common::PID(1, 0, 0);
            fr_str_joint = get_joint("chassis_fra_joint");
            jc->SetPositionPID(fr_str_joint->GetScopedName(), fr_pid);

            // Configure PID for axle joints
            fl_axle_joint = get_joint("chassis_flw_joint");
            fr_axle_joint = get_joint("chassis_frw_joint");

            // Configure PID for back left and right speed joints
            bl_pid = gazebo::common::PID(0.5, 0.01, 0.0);
            bl_axle_joint = get_joint("chassis_rlw_joint");
            jc->SetVelocityPID(bl_axle_joint->GetScopedName(), bl_pid);

            br_pid = gazebo::common::PID(0.5, 0.01, 0.0);
            br_axle_joint = get_joint("chassis_rrw_joint");
            jc->SetVelocityPID(br_axle_joint->GetScopedName(), br_pid);

            // Create publishers for odometry and pose
            odo_fl_pub = ros_node_->create_publisher<std_msgs::msg::Int32>("/odo_fl", 10);
            odo_fr_pub = ros_node_->create_publisher<std_msgs::msg::Int32>("/odo_fr", 10);
            cyberdrive_pub = ros_node_->create_publisher<cyber_interfaces::msg::CyberDriveStamped>(
                "/cmd_cyberdrive", 10);
            pose_pub = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", rclcpp::SensorDataQoS());
            odom_pub = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());

            // Subscribers for command inputs
            cyberdrive_sub = ros_node_->create_subscription<cyber_interfaces::msg::CyberDriveStamped>(
                "/cmd_cyberdrive", 2,
                std::bind(&CarGazeboPlugin::cyberdrive_callback, this, std::placeholders::_1));

            cmd_vel_sub = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 2, std::bind(&CarGazeboPlugin::twist_callback, this, std::placeholders::_1));
        }

        // Hook into the simulation update loop
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&CarGazeboPlugin::Update, this)); // Register the update method
    }

    /**
     * @brief Update the plugin state.
     *
     * This method is called every simulation iteration. It handles publishing the
     * current state of the model, updating joint positions, and broadcasting transforms.
     */
    void CarGazeboPlugin::Update() {
        auto cur_time = world_->SimTime(); // Get current simulation time
        if (last_sim_time_ == 0) {
            last_sim_time_ = cur_time;
            last_update_time_ = cur_time;
            return; // Skip the first update
        }

        // Publish state to ROS at specified intervals
        auto update_dt = (cur_time - last_update_time_).Double();
        if (update_dt * 1000 >= update_period_ms_) {
            // Publish pose
            auto pose = model_->WorldPose();
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = ros_node_->now(); // Set timestamp
            pose_msg.header.frame_id = "world"; // Set reference frame
            pose_msg.pose.position.x = pose.X(); // Set position
            pose_msg.pose.position.y = pose.Y();
            pose_msg.pose.position.z = pose.Z();
            pose_msg.pose.orientation.x = pose.Rot().X(); // Set orientation
            pose_msg.pose.orientation.y = pose.Rot().Y();
            pose_msg.pose.orientation.z = pose.Rot().Z();
            pose_msg.pose.orientation.w = pose.Rot().W();
            pose_pub->publish(pose_msg); // Publish pose message

            // Publish odometry
            nav_msgs::msg::Odometry odom_msg;
            auto linear_vel = model_->WorldLinearVel(); // Get linear velocity
            auto angular_vel = model_->WorldAngularVel(); // Get angular velocity
            odom_msg.header.stamp = ros_node_->now();
            odom_msg.header.frame_id = "world";
            odom_msg.pose.pose.position.x = pose.X();
            odom_msg.pose.pose.position.y = pose.Y();
            odom_msg.pose.pose.position.z = pose.Z();
            odom_msg.pose.pose.orientation.x = pose.Rot().X();
            odom_msg.pose.pose.orientation.y = pose.Rot().Y();
            odom_msg.pose.pose.orientation.z = pose.Rot().Z();
            odom_msg.pose.pose.orientation.w = pose.Rot().W();
            odom_msg.twist.twist.linear.x = linear_vel.X(); // Set linear velocity in odometry
            odom_msg.twist.twist.linear.y = linear_vel.Y();
            odom_msg.twist.twist.linear.z = linear_vel.Z();
            odom_pub->publish(odom_msg); // Publish odometry message

            // Create and send transform
            rclcpp::Time now = ros_node_->now();
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.stamp.sec = cur_time.sec;
            t.header.stamp.nanosec = cur_time.nsec;

            t.header.frame_id = "odom"; // Set the transform frame
            t.child_frame_id = "base_footprint"; // Child frame for the transform

            t.transform.translation.x = pose.X(); // Set translation
            t.transform.translation.y = pose.Y();
            t.transform.translation.z = pose.Z(); // Z is set to model height

            t.transform.rotation.x = pose.Rot().X(); // Set rotation
            t.transform.rotation.y = pose.Rot().Y();
            t.transform.rotation.z = pose.Rot().Z();
            t.transform.rotation.w = pose.Rot().W();

            tf_broadcaster_->sendTransform(t); // Broadcast the transform

            publish_state(); // Publish state (not defined in the original code)

            // Publish joint states
            auto msg = JointState{};
            msg.header.stamp.sec = cur_time.sec; // Timestamp for joint states
            msg.header.stamp.nanosec = cur_time.nsec;

            // Collect joint positions
            for (auto &j: joints_) {
                auto const &name = j.first;
                auto &joint = j.second.first;
                auto position = joint->Position(); // Get current position of the joint
                msg.name.push_back(name); // Add joint name
                msg.position.push_back(position); // Add joint position
            }
            joint_state_pub_->publish(msg); // Publish joint state message
            last_update_time_ = cur_time; // Update last update time
        }

        last_sim_time_ = cur_time; // Update last simulation time
    }

    // Register the plugin with Gazebo
    GZ_REGISTER_MODEL_PLUGIN(CarGazeboPlugin)
} // namespace car_gazebo_plugin
