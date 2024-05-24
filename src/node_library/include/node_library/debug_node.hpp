#ifndef RM_SENTRY_2024_DEBUG_NODE_
#define RM_SENTRY_2024_DEBUG_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/build_state.hpp"

namespace BehaviorTree{
    class DebugNode:public BT::SyncActionNode{
        private:
        public:

            DebugNode(const std::string&name, const BT::NodeConfig& config);
            int id;
            rclcpp::Time ti;
            static BT::PortsList providedPorts(){
                return{
                    BT::InputPort<int>("debug_output_id")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif