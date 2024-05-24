#ifndef RM_SENTRY_2024_ON_FIRE_NODE_
#define RM_SENTRY_2024_ON_FIRE_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "std_msgs/msg/int32.hpp"

namespace BehaviorTree{
    class OnFireNode:public BT::SyncActionNode{
        public:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_fire_pos;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_spin;
        rclcpp::Node::SharedPtr node1;

        OnFireNode(const std::string&name, const BT::NodeConfig& config);

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