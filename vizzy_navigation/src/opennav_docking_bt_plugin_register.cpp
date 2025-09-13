 /**
  * Copyright 2025, João Penha Lopes and João Zenário.
  * All rights reserved.
  */

#include "opennav_docking_bt/dock_robot.hpp"
#include "opennav_docking_bt/undock_robot.hpp"
#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder dock_builder =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_docking_bt::DockRobotAction>(name, "dock_robot", config);
    };

  factory.registerBuilder<opennav_docking_bt::DockRobotAction>("DockRobot", dock_builder);

  BT::NodeBuilder undock_builder =
    [&](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_docking_bt::UndockRobotAction>(name, "undock_robot", config);
    };
    
  factory.registerBuilder<opennav_docking_bt::UndockRobotAction>("UndockRobot", undock_builder);
}