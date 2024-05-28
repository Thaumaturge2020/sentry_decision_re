#ifndef RM_SENTRY_2024_PRIORITY_ONLY_NODE_
#define RM_SENTRY_2024_PRIORITY_ONLY_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/cam_command.hpp"
#include "decision_utils/string_parser.hpp"
#include "toml.hpp"

namespace BehaviorTree{
    class PriorityOnlyNode:public BT::SyncActionNode{
        public:
        PriorityOnlyNode(const std::string&name, const BT::NodeConfig& config);
        rclcpp::Publisher<robot_msgs::msg::CamCommand>::SharedPtr publisher_1;
        rclcpp::Node::SharedPtr node;
        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("autoaim_mode"),
                    BT::InputPort<std::string>("priority_level"),
                    BT::InputPort<std::string>("priority_type")
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif