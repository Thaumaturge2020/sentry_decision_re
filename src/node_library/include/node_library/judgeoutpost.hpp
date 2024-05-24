#ifndef RM_SENTRY_2024_JUDGEOUTPOST_
#define RM_SENTRY_2024_JUDGEOUTPOST_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include <nav_msgs/msg/odometry.hpp>


namespace BehaviorTree{
    
    class judgeoutpost:public BT::SyncActionNode{
        public:
        double limit_distance;
        judgeoutpost(const std::string&name, const BT::NodeConfig& config);
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_judpos_pos;
        rclcpp::Node::SharedPtr judgeoutp;
        void message_callback_judpos_pos(const nav_msgs::msg::Odometry &msg);
        geometry_msgs::msg::Point judpos_pos;
        geometry_msgs::msg::Point outpost_pos;
        static BT::PortsList providedPorts(){
                return {
                  //  BT::OutputPort<int>("area_choose"),
                    // BT::OutputPort<int>("id")

                };
            }

        BT::NodeStatus tick() override;
    };
   
}

#endif