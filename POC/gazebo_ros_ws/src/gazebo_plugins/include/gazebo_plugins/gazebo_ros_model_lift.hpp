// Copyright 2020 Space Engineering Research Center
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Author: Antariksh Narain

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_MODEL_LIFT_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_MODEL_LIFT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
    class GazeboRosModelLiftPrivate;

    /// A model lift plugin for engine prototyping. Subscribes to geometry_msgs/twist

    class GazeboRosModelLift : public gazebo::ModelPlugin
    {
    public:
        /// Constructor
        GazeboRosModelLift();

        /// Destructor
        ~GazeboRosModelLift();

    protected:
        // Documentation inherited
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

        // Documentation inherited
        void Reset() override;

    private:
        /// Private data pointer
        std::unique_ptr<GazeboRosModelLiftPrivate> impl_;
    };
} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__GAZEBO_ROS_MODEL_LIFT_HPP_
