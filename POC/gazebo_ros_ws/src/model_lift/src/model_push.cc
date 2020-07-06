#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "PIDController.cc"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public:
		#pragma region Public Data Members
		/// A pointer to the GazeboROS node.
  		gazebo_ros::Node::SharedPtr ros_node{nullptr};

		/// Odometry publisher
	    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

		/// True to publish IMU Data
		bool publish_imu;

		#pragma endregion

		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the model
			this->model = _parent;

			// Initialize ROS node
			this->ros_node = gazebo_ros::Node::Get(_sdf);

			// Get ROS QOS
			const gazebo_ros::QoS &qos = this->ros_node->get_qos();
			// Initialize the Publisher for position
			this->publish_imu = _sdf->Get<bool>("publish_imu", false).first;
			if(this->publish_imu)
			{
				this->odometry_pub_ = this->ros_node->create_publisher<nav_msgs::msg::Odometry>(
					"odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

				RCLCPP_INFO(
					this->ros_node_->get_logger(), "Advertise odometry on [%s]",
					this->odometry_pub_->get_topic_name());
			}
			// Configure Block constants
			//this->MAX_FORCE = 9.85; // Max force 12.0 N
			this->MAX_FORCE = 11.0; // Max force 12.0 N
			this->force_z = 0.0;
			this->increment = 0.01; // Precision of 0.01 N
		
			// Initialize controller
			//this->controller(0, 11.0, 0.1, 1, 1);
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
			
			// this->model->SetWorldTwist(ignition::math::v4::Vector3d(0.0, 0.0, 0.0),
			// 		ignition::math::v4::Vector3d(0.0, 1.5707, 0.0));
			// 	printf("Rotating Model");
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			// Apply a small linear velocity to the model.
			this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, 0.0, this->Thrust2(this->model->WorldPose().Pos(), this->expected_altitude)));
			//if(this->model->WorldPose().Pos().Z() >= (this->expected_altitude - 2))
			if(this->model->WorldPose().Pos().Z() >= (this->expected_altitude - 0))
			{
				this->expected_altitude = 0;
				printf("Coming Down!!!");
			}
		}

	protected:
		double Thrust(ignition::math::Vector3d imuData, double expectedAlti)
		{
			if(imuData.Z() < expectedAlti)
			{
				this->force_z += this->force_z < this->MAX_FORCE ? this->increment : 0;
			}
			else if(imuData.Z() > expectedAlti)
			{
				//this->force_z += this->force_z > 0 ? -this->increment : 0;
				this->force_z = 9.75;
			}
			else
			{
				printf("Hovering!!!");
			}
			printf("Force: %f N\t Pos(%f, %f %f)\n", this->force_z, imuData.X(), imuData.Y(), imuData.Z());
			return this->force_z;
		}

		double Thrust2(ignition::math::Vector3d imuData, double expectedAlti)
		{
			this->force_z = this->controller.Update(expectedAlti, imuData.Z());
			printf("Force: %f N\t Pos(%f, %f %f)\n", this->force_z, imuData.X(), imuData.Y(), imuData.Z());
			return this->force_z;
		}
		// void PIDController(double actualValue, double expectedValue)
		// {
		// 	const double KP = 0.1;
		// 	double error = expectedValue - actualValue;
			

		// }

		// Pointer to the model
	private:
		physics::ModelPtr model;

		// Pointer to the update event connection
	private:
		event::ConnectionPtr updateConnection;

		// Force (z) incrementer
	private:
		double MAX_FORCE;
		double force_z;
		double increment;
	
	private:
		PIDController controller = PIDController(9.6, 11.0, 1, 10, 10);
		bool rot_flag = true;
		double expected_altitude = 5.0;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo