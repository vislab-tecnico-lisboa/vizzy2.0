#include <opennav_docking_bt/dock_robot.hpp>
#include <opennav_docking_bt/undock_robot.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

// Ensure this function is exported with default visibility.
extern "C"
{
  void __attribute__((visibility("default"))) BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
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
}