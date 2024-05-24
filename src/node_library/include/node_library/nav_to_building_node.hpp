#ifndef RM_SENTRY_2024_NAV_TO_BUILDING_NODE_
#define RM_SENTRY_2024_NAV_TO_BUILDING_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/build_state_array.hpp"
#include "robot_msgs/msg/build_state.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "toml.hpp"

namespace BehaviorTree{
    class NavToBuildingNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Publisher<robot_msgs::msg::WalkCmd>::SharedPtr publisher_pos;
            rclcpp::Node::SharedPtr node1;
            NavToBuildingNode(const std::string&name, const BT::NodeConfig& config);
            geometry_msgs::msg::Point pos;
            std::vector<std::vector<double>> building_pos;
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("id")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif