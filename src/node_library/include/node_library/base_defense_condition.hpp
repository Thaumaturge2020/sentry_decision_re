#ifndef RM_SENTRY_2024_BASE_DEFENSE_CONDITION_
#define RM_SENTRY_2024_BASE_DEFENSE_CONDITION_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include <map>

namespace BehaviorTree{
    class BaseDefenseCondition:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node1;
        rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_blood;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_id;
        int self_id;
        bool base_flag,publish_flag;
        std::map<int,int> robot_list;

        BaseDefenseCondition(const std::string&name, const BT::NodeConfig& config);

        int gimbal_state;

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("true_or_false")
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif