/* Thanh Tran
* thanhtran@berkeley.edu
* This world plugin updates the score when a model lands in a zone
*/

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <gazebo/gazebo.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

namespace gazebo
{
  class UpdateGreenZoneScore : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      // Welcome message
      gzmsg << "Loading Green Zone plugin\n";
      std::cout << "Welcome to the Crater Landing Challenge!\n" << std::endl;
      std::cout << "Objective: Obtain the maximum points as possible by landing LEAPFROG in the center of the crater.\n" << std::endl;
      std::cout << "Green zone: 20 points" << std::endl;
      std::cout << "Blue zone: 10 points" << std::endl;
      std::cout << "Red zone: 5 points\n" << std::endl;
      
      // Transport initialization
      this->gzNode = transport::NodePtr(new transport::Node());
      this->gzNode->Init();

      // Subscribe to Green Zone ContainPlugin output
      std::string greenZoneTopic("green_zone_contain/contain");
      std::function<void(const ignition::msgs::Boolean &)> cb =
          [=](const ignition::msgs::Boolean &_msg){
        UpdateGreenZoneScore::OnContainPluginMsg(_msg);
      };
      const bool containSub = this->node.Subscribe(greenZoneTopic, cb);
      if (!containSub)
      {
        gzerr << "Failed to subscribe to [" << greenZoneTopic << "]\n";
      }
     
    }

    public: void OnContainPluginMsg(const ignition::msgs::Boolean &_msg)
    {
      // Initialize score
      int score = 0;
     
      // Update score if model lands in zone
      if (_msg.data())
      {
      	score += 20;
      	std::cout << "You've landed in the green zone!" << std::endl;
      	std::cout << "Score: " << score << std::endl;
      }
    }

    private: ignition::transport::Node node;
    private: transport::NodePtr gzNode;
    private: transport::PublisherPtr lightPub;
  };
  GZ_REGISTER_WORLD_PLUGIN(UpdateGreenZoneScore);
}  // namespace gazebo
