#ifndef GAZEBO_PLUGINS__ROS_ACTUATOR_MOVE_HPP_
#define GAZEBO_PLUGINS__ROS_ACTUATOR_MOVE_HPP_

#include <gazebo/common/Plugin.hh>

namespace gazebo_plugins
{
class GazeboRosActuatorMovePrivate;
class GazeboRosActuatorMove : public gazebo::ModelPlugin
{
public:
  // Constructor
  GazeboRosActuatorMove();

  // Descructor
  ~GazeboRosActuatorMove();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  //void Reset() override;

private:
  // Private data pointer
  std::unique_ptr<GazeboRosActuatorMovePrivate> impl_;

};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__ROS_ACTUATOR_MOVE_HPP_
