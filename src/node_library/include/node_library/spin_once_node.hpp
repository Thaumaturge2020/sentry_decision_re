#ifndef RM_SENTRY_2024_SPIN_ONCE_NODE_
#define RM_SENTRY_2024_SPIN_ONCE_NODE_


#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include"geometry_msgs/msg/point.h"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int32.hpp>
#include"robot_msgs/msg/build_state.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"


namespace BehaviorTree{
    class SpinOnceNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Time spin_ti;
            rclcpp::Node::SharedPtr node;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;
            rclcpp::Publisher<robot_msgs::msg::WalkCmd>::SharedPtr publisher_1;
            geometry_msgs::msg::Point self_position;
            SpinOnceNode(const std::string&name, const BT::NodeConfig& config);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("aerial_type"),
                    BT::OutputPort<int>("reset_aerial_type")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif