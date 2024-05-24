#ifndef RM_SENTRY_2024_COMPUTE_AREA_CHOOSE_
#define RM_SENTRY_2024_COMPUTE_AREA_CHOOSE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/build_state.hpp"

namespace BehaviorTree{
    class ComputeAreaChoose:public BT::SyncActionNode{
        private:
        public:

            ComputeAreaChoose(const std::string&name, const BT::NodeConfig& config);

            static BT::PortsList providedPorts()
            {
                return  
                {
                    BT::InputPort<int>("default_area_choose"),
                    BT::OutputPort<int>("area_choose"),
                };
            }

            BT::NodeStatus tick() override;
    };
}











#endif