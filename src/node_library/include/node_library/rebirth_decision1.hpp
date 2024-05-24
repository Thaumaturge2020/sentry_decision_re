#ifndef RM_SENTRY_2024_REBIRTH_DECISION_1_
#define RM_SENTRY_2024_REBIRTH_DECISION_1_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include <utility>
#include <vector>
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <toml.hpp>


namespace BehaviorTree{
    class RebirthDecision1:public BT::SyncActionNode{
        public:
        int num,blood,newblood,if_re;
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_blood;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_postion;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_point;
        rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_friend_position;
        std::vector<std::vector<std::pair<double,double>>> position;
        geometry_msgs::msg::Point a,c;
        geometry_msgs::msg::Point pub_pos;
        nav_msgs::msg::Odometry my_position;
        std::vector<std::pair<double,double>>friend_position;
        std::pair<double,double>en_position;
        double distance(double a,double b,double c,double d){return(a-b)*(a-b)+(c-d)*(c-d);}
        RebirthDecision1(const std::string&name, const BT::NodeConfig& config);
        void message_callback_blood(const std_msgs::msg::Int32 &msg);
        void message_callback_position(const nav_msgs::msg::Odometry &msg);
        void message_callback_friend_position(const robot_msgs::msg::AutoaimInfo &msg);

        static BT::PortsList providedPorts(){
                return {
                    BT::BidirectionalPort<int>("rebirth_decision_1"),
                    BT::BidirectionalPort<int>("if_rebirth"),
                    BT::BidirectionalPort<geometry_msgs::msg::Point>("pos")
                };
            }
        BT::NodeStatus tick() override;
    };
}

#endif