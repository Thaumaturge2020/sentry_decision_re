#ifndef RM_SENTRY_2024_CHECK_IF_BLOOD_INTERVAL_
#define RM_SENTRY_2024_CHECK_IF_BLOOD_INTERVAL_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include <map>

namespace BehaviorTree{
    class CheckIfBloodInterval:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node1;
        rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_blood;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_id;
        int self_id,target_id;
        int minimum_blood,maximum_blood;
        bool publish_flag;
        std::map<int,int> robot_list;

        CheckIfBloodInterval(const std::string&name, const BT::NodeConfig& config);

        int gimbal_state;

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("true_or_false"),
                    BT::InputPort<int>("target_id"),
                    BT::InputPort<int>("minimum_blood"),
                    BT::InputPort<int>("maximum_blood"),
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif