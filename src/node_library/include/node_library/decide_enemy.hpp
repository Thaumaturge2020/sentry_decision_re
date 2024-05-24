#ifndef RM_SENTRY_2024_DECIDE_ENEMY_
#define RM_SENTRY_2024_DECIDE_ENEMY_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "robot_msgs/msg/walk_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <set>
#include <utility>
#include "toml.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "decision_utils/id_mapping.hpp"

namespace BehaviorTree{
    class DecideEnemy:public BT::SyncActionNode{
        private:
        public:
            DecideEnemy(const std::string&name, const BT::NodeConfig& config);
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_enemy_pos;
            rclcpp::Subscription<robot_msgs::msg::RobotBloodInfo>::SharedPtr subscription_enemy_blood;
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_radar_info;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_my_pos;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_self_id;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_base_angle;
            rclcpp::Node::SharedPtr node;
            geometry_msgs::msg::Point My_pos;
            std::vector<robot_msgs::msg::RobotBattleState> robot_blood_array;
        
            std::vector<std::vector<std::pair<double,double> > > robot_map_field;
            std::vector<std::vector<int> > robot_map_connectivity;
            std::vector<std::vector<double> > robot_map_distance;
            std::vector<std::pair<double,double> > robot_centre_point;
            std::vector<double> robot_graph_height;
            std::pair<double,double> self_point,self_start_point;
            std::map<int,geometry_msgs::msg::Point> robot_pos_map;
            std::map<int,rclcpp::Time> robot_timer_map;
            std::map<int,int> robot_blood_map;


            std::vector< std::vector< int > > robot_priority;


            double base_angle,My_yaw,chase_distance_able,chase_distance_limit;
            int self_id;
            void message_callback_base_angle(const std_msgs::msg::Float64 &msg);
            void message_callback_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_radar_independent(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            void message_callback_self_id(const std_msgs::msg::Int32 &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::OutputPort<int>("target_enemy_id"),
                    BT::OutputPort<int>("chase_or_not"),
                    BT::OutputPort<std::vector<int> >("priority_output"),
                    BT::InputPort<std::string>("priority_input"),
                    BT::InputPort< double >("chase_distance_limit"),
                    BT::InputPort< double >("chase_distance_able")
            };
        }
            BT::NodeStatus tick() override;
    };
}

#endif