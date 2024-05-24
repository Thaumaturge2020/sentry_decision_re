#ifndef RM_SENTRY_2024_JUDGE_ENEMY_
#define RM_SENTRY_2024_JUDGE_ENEMY_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/build_state.hpp"
namespace BehaviorTree{
    class Judgenemy : public BT::SyncActionNode{
        private:
        public:
        Judgenemy(const std::string&name, const BT::NodeConfig& config);
        rclcpp::Subscription<robot_msgs::msg::BuildState>:: SharedPtr enemy_outpost_blood;
        rclcpp::Subscription<robot_msgs::msg::BuildState>:: SharedPtr my_outpost_blood;
        rclcpp::Node::SharedPtr Judgenemy_0;
        
        int enemy_outpost,my_outpost;

        void message_callback_enemy_oupost_blood(const robot_msgs::msg::BuildState &msg);
        void message_callback_my_oupost_blood(const robot_msgs::msg::BuildState &msg);

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("my_outpost_blood_lowerbound"),
                    BT::InputPort<int>("my_outpost_blood_upperbound"),
                    BT::InputPort<int>("enemy_outpost_blood_lowerbound"),
                    BT::InputPort<int>("enemy_outpost_blood_upperbound"),
                };
            }
        BT::NodeStatus tick() override;
    };
}
#endif