#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ActuatorMove : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->actuator = model->GetLink("inner_cylinder");

      // Listen to the update event. This event is broadcated evry simulation
      // iteration
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ActuatorMove::OnUpdate, this)
      );
    }

    // Called by the world upate start event
    public: void OnUpdate()
    {
      if (this->model->WorldPose().Pos().X() <= 0) {
        this->velocity_x = 0.5;
      } else if (this->model->WorldPose().Pos().X() >= 0.5) {
        this->velocity_x = -0.5;
      }
      // velocity defined on x axis as set by joint in actuator
      this->actuator->SetLinearVel(ignition::math::Vector3d(this->velocity_x, 0, 0));
      // velocity defined on x axis as set by joint in actuator
      this->model->SetLinearVel(ignition::math::Vector3d(this->velocity_x, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    private: physics::LinkPtr actuator;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:
      double velocity_x;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActuatorMove)
}
