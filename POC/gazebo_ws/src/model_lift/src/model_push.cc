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
			//this->MAX_FORCE = 9.85; // Max force 12.0 N
			this->MAX_FORCE = 11.0; // Max force 12.0 N
			this->force_z = 0.0;
			this->force_y = 0.0;
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
		// void OnUpdate2()
		// {
		// 	// Apply a small linear velocity to the model.
		// 	this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, 0.0, this->Thrust2(this->model->WorldPose().Pos(), this->expected_altitude)));
		// 	//if(this->model->WorldPose().Pos().Z() >= (this->expected_altitude - 2))
		// 	if(this->model->WorldPose().Pos().Z() >= (this->expected_altitude - 0.2))
		// 	{
		// 		this->expected_altitude = 0;
		// 		printf("Coming Down!!!");
		// 	}
		// }

		void OnUpdate()
		{
			// Apply a small linear velocity to the model.
			// Stage 0: Lift off
			this->DirectionThrust(this->model->WorldPose().Pos(), this->expected_altitude);
			if (this->stages == 0)
			{
				this->expected_altitude = ignition::math::Vector3d(0.0, 0.0, 5.0);
				this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, this->force_y, this->force_z));
			}
			// Stage 1: Move with TVC
			else if (this->stages == 1)
			{
				this->expected_altitude = ignition::math::Vector3d(0.0, 2.0, 5.0);
				this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, this->force_y, this->force_z));
			}
			// Stage 2: Land
			else if (this->stages == 2)
			{
				this->expected_altitude = ignition::math::Vector3d(0.0, 2.0, 0.0);
				this->model->GetLink("link")->SetForce(ignition::math::Vector3d(0.0, this->force_y, this->force_z));
			}

			//if(this->model->WorldPose().Pos().Z() >= (this->expected_altitude - 2))
			if (this->model->WorldPose().Pos().Z() >= (this->expected_altitude.Z() - 0.2))
			{
				if (this->model->WorldPose().Pos().Y() < 3.0)
				{
					stages = 1;
					printf("Moving Forward!");
				}
				if (this->model->WorldPose().Pos().Y() >= 3.0)
				{
					stages = 2;
					printf("Landing Down!");
				}
			}
		}

	protected:
		// double Thrust(ignition::math::Vector3d imuData, double expectedAlti)
		// {
		// 	if(imuData.Z() < expectedAlti)
		// 	{
		// 		this->force_z += this->force_z < this->MAX_FORCE ? this->increment : 0;
		// 	}
		// 	else if(imuData.Z() > expectedAlti)
		// 	{
		// 		//this->force_z += this->force_z > 0 ? -this->increment : 0;
		// 		this->force_z = 9.75;
		// 	}
		// 	else
		// 	{
		// 		printf("Hovering!!!");
		// 	}
		// 	printf("Force: %f N\t Pos(%f, %f %f)\n", this->force_z, imuData.X(), imuData.Y(), imuData.Z());
		// 	return this->force_z;
		// }

		// double Thrust2(ignition::math::Vector3d imuData, double expectedAlti)
		// {
		// 	this->force_z = this->controller.Update(expectedAlti, imuData.Z());
		// 	printf("Force: %f N\t Pos(%f, %f %f)\n", this->force_z, imuData.X(), imuData.Y(), imuData.Z());
		// 	return this->force_z;
		// }

		// double Thrust3(ignition::math::Vector3d imuData, double expectedAlti)
		// {

		// 	this->force_z = this->controller.Update(expectedAlti, imuData.Z());
		// 	printf("Force: %f N\t Pos(%f, %f %f)\n", this->force_z, imuData.X(), imuData.Y(), imuData.Z());
		// 	return this->force_z;
		// }
		double DirectionThrust(ignition::math::Vector3d imuData, ignition::math::Vector3d expectedPosition)
		{
			this->force_z = this->controllerZ.Update(expectedPosition.Z(), imuData.Z());
			this->force_y = this->controllerY.Update(expectedPosition.Y(), imuData.Y());
			printf("Force: %.2f, %.2f N\t Pos(%.2f, %.2f %.2f) \t ExpPos(0.0, %.2f, %.2f)\n", this->force_z, this->force_y, imuData.X(), 
				imuData.Y(), imuData.Z(), expectedPosition.Y(), expectedPosition.Z());
		}

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
		double force_y;
		double increment;

	private:
		PIDController controllerZ = PIDController(9.6, 11.0, 1, 10, 10);
		PIDController controllerY = PIDController(-1.0, 1.0, 0.8, 0.3, 0.1, 1, 10, 10);
		bool rot_flag = true;
		//double expected_altitude = 5.0;
		ignition::math::Vector3d expected_altitude = ignition::math::Vector3d(0.0, 0.0, 5.0);

	private:
		int stages = 0;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo