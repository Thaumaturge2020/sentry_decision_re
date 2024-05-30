#ifndef RM_SENTRY_2024_RESURRECTION_NODE_
#define RM_SENTRY_2024_RESURRECTION_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "decision_utils/judging_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/sentry_referee_decision.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "toml.hpp"

namespace BehaviorTree{
    class ResurrectionNode:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_blood;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_id;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_self_position;
        rclcpp::Publisher<robot_msgs::msg::SentryRefereeDecision>::SharedPtr publisher;
        int self_id;
        bool base_flag,publish_flag;
        std::pair<double,double> self_position;
        std::pair<double,double> bias_position;
        bool recover_if;
        int self_last_blood;
        rclcpp::Time recover_start_time;
        rclcpp::Time occupy_start_time;
        int blood_limit,time_limit;
        double given_distance_limit;
        geometry_msgs::msg::Point navigation_point_input;

        ResurrectionNode(const std::string&name, const BT::NodeConfig& config);

        int gimbal_state;

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("area_choose"),
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif