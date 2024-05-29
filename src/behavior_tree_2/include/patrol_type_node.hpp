#ifndef RM_SENTRY_2024_PATROL_TYPE_NODE_
#define RM_SENTRY_2024_PATROL_TYPE_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"

namespace BehaviorTree{
    class PatrolTypeNode:public BT::SyncActionNode{
        public:

        PatrolTypeNode(const std::string&name, const BT::NodeConfig& config);

        int patrol_type;

        static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<int>("val_port"),
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif