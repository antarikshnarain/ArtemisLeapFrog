/* ---------------------------------------------------------------
 * Copyright 2020 Space Engineering Research Center
 * Author: Antariksh Narain
 * 
 * Description:
 * 
 * --------------------------------------------------------------- */

#ifndef GAZEBO_PLUGINS__MODEL_COLD_GAS_THRUSTER_HPP
#define GAZEBO_PLUGINS__MODEL_COLD_GAS_THRUSTER_HPP

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
    class GazeboRosColdGasThrusterPrivate;

    /// A model lift plugin for engine prototyping. Subscribes to geometry_msgs/twist

    class GazeboRosColdGasThruster : public gazebo::ModelPlugin
    {
    public:
        /// Constructor
        GazeboRosColdGasThruster();

        /// Destructor
        ~GazeboRosColdGasThruster();

    protected:
        // Documentation inherited
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        // Documentation inherited
        void Reset() override;

    private:
        /// Private data pointer
        std::unique_ptr<GazeboRosColdGasThrusterPrivate> impl_;
    };
} // namespace gazebo_plugins

#endif