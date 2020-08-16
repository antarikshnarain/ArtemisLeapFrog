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
  class UpdateBlueZoneScore : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      gzmsg << "Loading Blue Zone plugin\n";
      // Transport initialization
      this->gzNode = transport::NodePtr(new transport::Node());
      this->gzNode->Init();

      // Subscribe to Blue Zone ContainPlugin output
      std::string blueZoneTopic("blue_zone_contain/contain");
      std::function<void(const ignition::msgs::Boolean &)> cb =
          [=](const ignition::msgs::Boolean &_msg){
        UpdateBlueZoneScore::OnContainPluginMsg(_msg);
      };
      const bool containSub = this->node.Subscribe(blueZoneTopic, cb);
      if (!containSub)
      {
        gzerr << "Failed to subscribe to [" << blueZoneTopic << "]\n";
      }
     
    }

    public: void OnContainPluginMsg(const ignition::msgs::Boolean &_msg)
    {
      // Initialize score
      int score = 0;
     
      // Update score if model lands in zone
      if (_msg.data())
      {
      	score += 10;
      	std::cout << "You've landed in the blue zone!" << std::endl;
      	std::cout << "Score: " << score << std::endl;
      }
    }

    private: ignition::transport::Node node;
    private: transport::NodePtr gzNode;
    private: transport::PublisherPtr lightPub;
  };
  GZ_REGISTER_WORLD_PLUGIN(UpdateBlueZoneScore);
}  // namespace gazebo
