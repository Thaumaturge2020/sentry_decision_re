#include "node_library/spin.hpp"

namespace BehaviorTree{

    Spin::Spin(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "spin";
                    node = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_enemy_pos = node->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&Spin::message_callback_enemy_pos,this,std::placeholders::_1));
                    my_pos = node->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&Spin::message_callback_my_pos,this,std::placeholders::_1));
                    
                    pub_spin_able = node->create_publisher<std_msgs::msg::Int64>("decision2basespin", 10);
                    sub_build_state = node->create_subscription<robot_msgs::msg::BuildState>("sentry/build_state",10,std::bind(&Spin::message_callback_build_state,this,std::placeholders::_1));
                    sub_self_blood = node->create_subscription<robot_msgs::msg::RobotBloodInfo>("sentry/team_blood",10,std::bind(&Spin::message_callback_self_blood,this,std::placeholders::_1));
                }

    void Spin::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    void Spin::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        My_pos = msg.pose.pose.position;
        return;
    }

    void Spin::message_callback_build_state(const robot_msgs::msg::BuildState &msg){
        build_state.blood = msg.blood;
        return;
    }

    void Spin::message_callback_self_blood(const robot_msgs::msg::RobotBloodInfo &msg){
        self_blood.data = msg.data;
        return;
    }

    BT::NodeStatus Spin::tick(){
        RCLCPP_INFO(rclcpp::get_logger("base_attack_node"),"I'm ticked");
        rclcpp::spin_some(node);
        size_t array_size = robot_pos_array.size();
        if(build_state.blood != 0||array_size == 0){
            spin_able.data = 0;
        }
        else{
            if(My_pos.x <= 7){
                spin_able.data = 5;
            }
            else
                spin_able.data = 0;
        }
        pub_spin_able->publish(spin_able);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Spin>("SpinNode");
// }