#ifndef RM_SENTRY_2024_NAV_HEAL_NODE_
#define RM_SENTRY_2024_NAV_HEAL_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "toml.hpp"
#include <utility>

namespace BehaviorTree{
    class nav_heal_node:public BT::SyncActionNode{
        public:
        rclcpp::Node::SharedPtr node1;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_heal_point;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_refresh_blood;
        rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_blood_info;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_robot_id;
        nav_heal_node(const std::string&name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        geometry_msgs::msg::Point heal_position;
        bool check_blood(bool type);
        bool initial_type;
        std::vector<int> sentry_blood_interval;
        int sentry_blood_refresh_limit;
        int robot_id;
        int refresh_blood_count;
        robot_msgs::msg::RobotBloodInfo blood_info,blood_info_last;
        void refresh_robot_blood(int id);
        void blood_callback(const robot_msgs::msg::RobotBloodInfo &msg);
        void robot_id_callback(const std_msgs::msg::Int32 &msg);
        static BT::PortsList providedPorts(){
            return{
                BT::OutputPort<geometry_msgs::msg::Point>("heal_navigation_point"),
                BT::InputPort<int>("interval_0"),
                BT::InputPort<int>("interval_1")
            };
        }
    };
}

#endif