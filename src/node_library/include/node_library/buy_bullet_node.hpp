#ifndef RM_SENTRY_2024_BUY_BULLET_NODE_
#define RM_SENTRY_2024_BUY_BULLET_NODE_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "robot_msgs/msg/build_state.hpp"
#include "robot_msgs/msg/sentry_referee_decision.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "toml.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>


namespace BehaviorTree{
    class BuyBulletNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Publisher<robot_msgs::msg::SentryRefereeDecision>::SharedPtr publisher;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;
            rclcpp::Node::SharedPtr node1;
            rclcpp::Time start_reach_Time;

            std::pair<double,double> bias_position,self_position;

            geometry_msgs::msg::Point navigation_point;

            int able_buy;

            BuyBulletNode(const std::string&name, const BT::NodeConfig& config);

            static BT::PortsList providedPorts()
            {
                return  
                {
                    BT::InputPort<int>("bullet_buy_num"),
                    BT::InputPort<int>("area_id"),
                    BT::OutputPort<int>("reset_aerial_type")
                };
            }

            BT::NodeStatus tick() override;
    };
}


#endif