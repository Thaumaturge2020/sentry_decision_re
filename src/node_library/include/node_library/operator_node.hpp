#ifndef RM_SENTRY_2024_OPERATOR_NODE_
#define RM_SENTRY_2024_OPERATOR_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/aerial_commands.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "decision_utils/id_mapping.hpp"

namespace BehaviorTree{
    class OperatorNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AerialCommands>::SharedPtr subscription_operator_cmd;
            rclcpp::Node::SharedPtr node1;
            geometry_msgs::msg::Point navigate_point;
            int flag,aerial_type;
            OperatorNode(const std::string&name, const BT::NodeConfig& config);
            void message_callback_operator_cmd(const robot_msgs::msg::AerialCommands &msg);
            int target_enemy;
            static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<int>("expected_operator_mode"),
                    BT::OutputPort<int>("expected_enemy"),
                    BT::OutputPort<geometry_msgs::msg::Point>("expected_place"),
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif