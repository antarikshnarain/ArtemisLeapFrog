#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_plugins/gazebo_ros_actuator_move.hpp>
#include <geometry_msgs/msg/twist.hpp>

// #include <ignition/math/Vector3.hh>

namespace gazebo_plugins
{
class GazeboRosActuatorMovePrivate
{
public:
  /// Indicate which actuator
  enum
  {
    /// X axis
    X_ACC,

    /// Y axis
    Y_ACC
  };

  /// Callback to be called at every simulation iteration
  /// \param[in] _info Updated simulation info
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

  /// A pointer to the GazeboROS node
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to model
  gazebo::physics::ModelPtr model_;

  /// Pointer to actuators
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  // Max steering angle
  double max_speed_ = 0;

  /// Linear velocity in X received on command (m/s).
  double target_linear_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};
};

GazeboRosActuatorMove::GazeboRosActuatorMove()
: impl_(std::make_unique<GazeboRosActuatorMovePrivate>())
{
}

GazeboRosActuatorMove::~GazeboRosActuatorMove()
{
}

void GazeboRosActuatorMove::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profile
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Two actuators for x and y axes respectively
  impl_->joints_.resize(2);

  // get inner cylinder to translate
  auto actuator_joint = _sdf->Get<std::string>("actuator_joint", "actuator_joint").first;
  impl_->joints_[GazeboRosActuatorMovePrivate::X_ACC] = _model->GetJoint(actuator_joint);
  // TODO: do error check for joint existence; return and reset if failed

  impl_->max_speed_ = _sdf->Get<double>("max_speed", 0.1).first;

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosActuatorMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosActuatorMovePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosActuatorMovePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> lock(lock_);

  // Set velocity of actuator
  auto target_linear = ignition::math::clamp(target_linear_, -max_speed_, max_speed_);

  // TODO: Set bounds for hitting maxmimum extension

  joints_[X_ACC]->SetVelocity(0, target_linear);
}

void GazeboRosActuatorMovePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_linear_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
  RCLCPP_INFO(
    ros_node_->get_logger(), "Received new linear value [%f]", target_linear_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosActuatorMove)
}  // namespace gazebo_plugins
