#ifndef RM_SENTRY_2024_STAY_ORI_NODE_
#define RM_SENTRY_2024_STAY_ORI_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "toml.hpp"
#include <cstring>
#include <utility>

namespace BehaviorTree{
    class StayOriNode:public BT::SyncActionNode{
        private:
        public:
            StayOriNode(const std::string&name, const BT::NodeConfig& config);
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odometry_subscription;
            rclcpp::Node::SharedPtr node_stay_ori;
           // std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            rclcpp::Time start_time,now_time;
            double during;
            geometry_msgs::msg::Point pose_stamp;
            void message_callback_odometry(const nav_msgs::msg::Odometry &msg);
            std::pair<double,double> init_pos;
            static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<geometry_msgs::msg::Point>("now_navigation_point")
                };
            }
            BT::NodeStatus tick() override;
           
    };
}

#endif