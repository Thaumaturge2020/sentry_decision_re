#ifndef RM_SENTRY_2024_DEFENCE_BUILDING_
#define RM_SENTRY_2024_DEFENCE_BUILDING_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/build_state_array.hpp"
#include "robot_msgs/msg/build_state.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "toml.hpp"
#include <vector>

namespace BehaviorTree{
    class DefenceBuildingNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_build_state;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_build_id;
            rclcpp::Node::SharedPtr node1;
            DefenceBuildingNode(const std::string&name, const BT::NodeConfig& config);
            std::vector<robot_msgs::msg::RobotBattleState> build_state_array;
            void message_callback_build_state(const robot_msgs::msg::RobotBloodInfo &msg);
            void message_callback_self_id(const std_msgs::msg::Int32 &msg);
            int self_id,build_blood_threshold;
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("building_id"),
                    BT::InputPort<int>("building_type"),
                    BT::InputPort<int>("building_blood_threshold")  
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif