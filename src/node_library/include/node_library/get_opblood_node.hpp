#ifndef RM_SENTRY_2024_GET_OPBLOOD_NODE_
#define RM_SENTRY_2024_GET_OPBLOOD_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/build_state.hpp"

namespace BehaviorTree{
    class GetOpbloodNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Node::SharedPtr node1;
            rclcpp::Subscription<robot_msgs::msg::BuildState>::SharedPtr subscription_enemy_op_info;
            rclcpp::Subscription<robot_msgs::msg::BuildState>::SharedPtr subscription_my_op_info;
            GetOpbloodNode(const std::string&name, const BT::NodeConfig& config);
            int enemy_opblood,my_opblood;

            void enemyop_info_recevice_callback(const robot_msgs::msg::BuildState &msg);
            void myop_info_recevice_callback(const robot_msgs::msg::BuildState &msg);

            static BT::PortsList providedPorts(){
                return{
                    BT::OutputPort<int>("enemy_outpost_blood"),
                    BT::OutputPort<int>("my_outpost_blood")
                };
            }
            BT::NodeStatus tick() override;
    };
}





#endif