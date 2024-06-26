#ifndef RM_SENTRY_2024_TIME_BEGIN_
#define RM_SENTRY_2024_TIME_BEGIN_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"

namespace BehaviorTree{
    class TimeBegin:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<std_msgs::msg::Int32> ::SharedPtr time_begin_subscription;
            rclcpp::Node::SharedPtr node_time_begin;
            TimeBegin(const std::string&name, const BT::NodeConfig& config);
           // std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            rclcpp::Time start_time,now_time;
            double during;
            int time_stamp;
            void message_callback_time_begin(const std_msgs::msg::Int32 &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::BidirectionalPort<double>("time_during"),
                    BT::InputPort<int>("true_or_false")
                };
            }
            BT::NodeStatus tick() override;
           
    };
}

#endif