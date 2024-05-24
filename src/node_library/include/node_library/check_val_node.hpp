#ifndef RM_SENTRY_2024_CHECK_VAL_NODE_
#define RM_SENTRY_2024_CHECK_VAL_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "std_msgs/msg/int32.hpp"

namespace BehaviorTree{
    class CheckValNode:public BT::SyncActionNode{
        public:
        int val_desire;

        CheckValNode(const std::string&name, const BT::NodeConfig& config);

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("val_port"),
                    BT::InputPort<int>("val_desire")
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif