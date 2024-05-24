#ifndef RM_SENTRY_2024_GIMBAL_CHOOSE_BUILDING_
#define RM_SENTRY_2024_GIMBAL_CHOOSE_BUILDING_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/cam_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "toml.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "decision_utils/id_mapping.hpp"

namespace BehaviorTree{
    class GimbalChooseBuildingNode:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_base_angle;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_cam_angle;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_base2cam_angle;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_autoaim_able;
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_my_pos;
            rclcpp::Publisher<robot_msgs::msg::CamCommand>::SharedPtr publisher_enemy_info;

            rclcpp::Node::SharedPtr gimbal_choose_building_node;
            GimbalChooseBuildingNode(const std::string&name, const BT::NodeConfig& config);
            double base_angle,cam_angle,base2cam_angle;
            std::vector<robot_msgs::msg::RobotInfo> enemy_pos;
            std::vector<std::vector<double> > building_pos;

            std::pair<double,double> self_start_point,My_pos;
            
            int autoaim_able;
            void message_callback_base_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_cam_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_base2cam_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_autoaim_able(const std_msgs::msg::Int32 &msg);
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("id"),
                    BT::InputPort<int>("shooting_speed")
                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif