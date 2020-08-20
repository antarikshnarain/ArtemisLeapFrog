/* ---------------------------------------------------------------
 * 
 * Author: Antariksh Narain
 * Organization: Space Engineering Research Center
 * 
 * Description:
 * 
 * --------------------------------------------------------------- */

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_model_lift.hpp>
#include <utilities/PIDController.hpp>
#include "gazebo_msgs/msg/world_pose.hpp" // /msg/world_pose.hpp>
#include "gazebo_msgs/msg/ColdGasThrusterState.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

using namespace leapfrog_utilities;

namespace gazebo_plugins
{
    class GazeboRosColdGasThrusterPrivate
    {
    protected:
        /// Moves the rotating servo to desired position
        /// \param[in] servo angle between -1.57 to 1.57 rad
        bool MoveServo(double expected_angle);

        /// Applies force from thruster arm
        bool ApplyThrust();

    public:
        /// Indicates which link
        enum
        {
            /// Engine Link
            LINK_ENGINE
        };
        /// Callback to be called at every simulation iteration.
        /// \param[in] _info Updated simulation info.
        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        /// Callback when a velocity command is received.
        /// \param[in] _msg Twist command message.
        void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

        /// Update odometry according to world
        void UpdateOdometryWorld();

        /// Publish odometry messages
        /// \param[in] _current_time Current simulation time
        void PublishOdometryMsg(const gazebo::common::Time &_current_time);

        /// Publish world position messages
        /// \param[in] _current_time Current simulation time
        void PublishWorldPoseMsg(const gazebo::common::Time &_current_time);

        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Subscriber to command velocities
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        /// Odometry publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

        /// World Pose publisher
        rclcpp::Publisher<gazebo_msgs::msg::WorldPose>::SharedPtr worldpose_pub;

        /// Pointers to links.
        std::vector<gazebo::physics::LinkPtr> links_;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        /// Pointer to model.
        gazebo::physics::ModelPtr model_;

        /// To broadcast TFs
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

        /// Protect variables accessed on callbacks.
        std::mutex lock_;

        /// Max altitude limit for Geo Fencing
        double max_altitude{0.0};

        /// Geo Fencing limit in X axis
        ignition::math::Vector2d geofence_X;

        /// Geo Fencing limit in Y axis
        ignition::math::Vector2d geofence_Y;

        /// Geo Fencing limit in Z axis
        ignition::math::Vector2d geofence_Z;


        /// Update period in seconds.
        double update_period_;

        /// Last update time.
        gazebo::common::Time last_update_time_;

        /// Odometry frame ID
        std::string odometry_frame_;

        /// Keep latest odometry message
        nav_msgs::msg::Odometry odom_;

        /// Keep latest world pose message
        gazebo_msgs::msg::WorldPose pose_;

        /// Robot base frame ID
        std::string robot_base_frame_;

        /// True to publish odometry messages.
        bool publish_odom_;

        bool publish_world_pose;

        /// Covariance in odometry
        double covariance_[3];

        /// PID control for Engine thrust vector
        gazebo::common::PID pid_engine_thrust_;

        /// PID controller 
        PIDController controller = PIDController(9.6, 11.0, 1, 10, 10);
		
        /// Rotation Flag
        bool rot_flag = true;
    };

    GazeboRosModelLift::GazeboRosModelLift()
        : impl_(std::make_unique<GazeboRosModelLiftPrivate>())
    {
    }

    GazeboRosModelLift::~GazeboRosModelLift()
    {
    }

    void GazeboRosModelLift::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        /**Initialize environment**/
        impl_->model_ = _model;

        auto world = impl_->model_->GetWorld();
        auto physicsEngine = world->Physics();
        physicsEngine->SetParam("friction_model", std::string("cone_model"));

        /**Initialize ROS node**/
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

        // Get QoS profiles
        const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

        /**Load Properties from world**/
        double max_force = _sdf->Get<double>("max_force", 0.0).first;
        double force_duration = _sdf->Get<double>("force_duration", 0.0).first;
        double capacity = _sdf->Get<double>("capacity", 0.0).first;
        double servo_delay = _sdf->Get<double>("servo_delay", 0.0).first;
        impl_->max_altitude = max_altitude;

        impl_->last_update_time_ = _model->GetWorld()->SimTime();

        // Odometry
        impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
        impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
        
        std::string name_list_links[] = {"link_base", "thruster_base", "link_arm", "thruster_arm"};
        for(int i=0;i<4;i+=2)
        {
            auto model_link = _sdf->Get<std::string>(name_list_links[i], name_list_links[i+1]);
            impl_->links_[i/2] = _model->GetLink(model_link);
            if (!impl_->links_[i/2]) 
            {
            RCLCPP_ERROR(
                impl_->ros_node_->get_logger(),
                "Link [%s] not found, plugin will not work.", model_link.c_str()
            );
            impl_->ros_node_.reset();
            return;
            }
        }

        std::string name_list_joints[] = {"joint_rotate", "joint_thrust_rotate"};
        for(int i=0;i<2;i+=2)
        {
            auto model_joint = _sdf->Get<std::string>(name_list_joints[i], name_list_joints[i+1]);
            impl_->joints_[i/2] = _model->GetLink(model_joint);
            if (!impl_->joints_[i/2]) 
            {
            RCLCPP_ERROR(
                impl_->ros_node_->get_logger(),
                "Link [%s] not found, plugin will not work.", model_joint.c_str()
            );
            impl_->ros_node_.reset();
            return;
            }
        }
//        impl_->links_[GazeboRosModelLiftPrivate::LINK_ENGINE] = _model->GetLink(model_engine_link);

        /**Publisher and Subscriber**/
        impl_->cmd_thrust = impl_->ros_node_->create_subscription<geometry_msgs::msg::ColdGasThrusterState>(
            "cmd_thrust", qos.get_subscription_qos("cmd_thrust", rclcpp::QoS(1)),
            std::bind(&GazeboRosColdGasThrusterPrivate::MoveServo, impl_.get(), std::placeholders::_1));

        RCLCPP_INFO(
            impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_thrust->get_topic_name());

        // Advertise System stats
        impl_->system_stats = _sdf->Get<bool>("publish_system_stats", false).first;
        if (impl_->system_stats)
        {
            impl_->publish_system_stats = impl_->ros_node_->create_publisher<geometry_msgs::msg::ThrusterStats>(
                "system_stats", qos.get_publisher_qos("system_stats", rclcpp::QoS(1)));
            RCLCPP_INFO(
                impl_->ros_node_->get_logger(), "Advertise System stats on [%s]",
                impl_->publish_system_stats->get_topic_name());
        }

        /**Initialize Other variables**/
        // auto pose = impl_->model_->WorldPose();
        // impl_->odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
        // impl_->odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(
        //     pose.Rot());

        // impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
        // impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
        // impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;
        //impl_->covariance_[0] = 
        // Listen to the update event (broadcast every simulation iteration)
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosModelLiftPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
    }

    void GazeboRosModelLift::Reset()
    {
        impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
    }

    void GazeboRosModelLiftPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
    {
        std::lock_guard<std::mutex> lock(lock_);

        double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

        // Update odom
        UpdateOdometryWorld();

        if (seconds_since_last_update < update_period_)
        {
            return;
        }

        if (publish_odom_)
        {
            PublishOdometryMsg(_info.simTime);
        }

        if (publish_world_pose)
        {
            PublishWorldPoseMsg(_info.simTime);
        }

        auto current_position = links_[LINK_ENGINE]->WorldPose();
        double force_z = this->Thrust(current_position.Pos());
        links_[LINK_ENGINE]->SetForce(ignition::math::Vector3d(0, 0, force_z));

        last_update_time_ = _info.simTime;
    }

    void GazeboRosModelLiftPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(lock_);
    }

    void GazeboRosModelLiftPrivate::PublishWorldPoseMsg(const gazebo::common::Time &_current_time)
    {
        auto pos = links_[LINK_ENGINE]->WorldPose().Pos();
        pose_.pose.position.x = pos.X();
        pose_.pose.position.y = pos.Y();
        pose_.pose.position.z = pos.Z();
        this->worldpose_pub->publish(pose_);
        //this->worldpose_pub->publish(pose_);
    }

    double GazeboRosModelLiftPrivate::Thrust(ignition::math::Vector3d imuData)
    {
        return this->controller.Update(this->max_altitude, imuData.Z());
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosModelLift)
} // namespace gazebo_plugins
