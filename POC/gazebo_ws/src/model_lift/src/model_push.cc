#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "PIDController.cc"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// Store the pointer to the model
			this->model = _parent;

			// Configure Block constants
			this->MAX_FORCE = 9.85; // Max force 12.0 N
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
			//this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
			//this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, 0.0, this->force_z));
			this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, 0.0, this->Thrust2(this->model->WorldPose().Pos(), 5.0)));
			// roll at 4.5
			if(this->model->WorldPose().Pos().Z() > 4.5 && rot_flag)
			{
				this->model->SetWorldTwist(ignition::math::v4::Vector3d(0.0, 0.0, 0.0),
					ignition::math::v4::Vector3d(0.0, 0.17453, 0.0));
				printf("Rotating Model\n");
				rot_flag = false;
				// this->model->SetWorldTwist(ignition::math::v4::Vector3d(0.0, 0.0, 0.0),
				// 	ignition::math::v4::Vector3d(0.0, 0.0, 0.0));
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
		PIDController controller = PIDController(9.5, 11.0, 0.1, 1, 10);
		bool rot_flag = true;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo