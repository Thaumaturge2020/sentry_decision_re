//#include"judgenemy.hpp"
#include "node_library/judge_enemy.hpp"
namespace BehaviorTree{
    Judgenemy::Judgenemy(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {

                    std::stringstream ss;
                    ss << "judgenemy";
        Judgenemy_0 = rclcpp::Node::make_shared(ss.str().c_str());
        //敌我双方前哨战信息，名字没有写
        enemy_outpost_blood = Judgenemy_0->create_subscription<robot_msgs::msg::BuildState>("enemy_outpost_blood", 10, std::bind(&Judgenemy::message_callback_enemy_oupost_blood, this, std::placeholders::_1));
        my_outpost_blood = Judgenemy_0->create_subscription<robot_msgs::msg::BuildState>("my_outpost_blood", 10, std::bind(&Judgenemy::message_callback_my_oupost_blood, this, std::placeholders::_1));
        enemy_outpost = -1;
        my_outpost = -1;
    }
    void Judgenemy::message_callback_enemy_oupost_blood(const robot_msgs::msg::BuildState &msg){
        enemy_outpost=msg.blood;
        setOutput<int>("enemy_outpost_blood",enemy_outpost);

    }
    void Judgenemy::message_callback_my_oupost_blood(const robot_msgs::msg::BuildState &msg){
        my_outpost=msg.blood;
        setOutput<int>("my_outpost_blood",my_outpost);
    }
    BT::NodeStatus Judgenemy::tick(){
        //可以加一个get黑板是否空白来返回failure
        rclcpp::spin_some(Judgenemy_0);
        int my_outpost_blood_lowerbound,
            my_outpost_blood_upperbound,
            enemy_outpost_blood_lowerbound,
            enemy_outpost_blood_upperbound;
        if(!getInput<int>("my_outpost_blood_lowerbound",my_outpost_blood_lowerbound)) {RCLCPP_INFO(rclcpp::get_logger("outpost_judge"),"my outpost lower"); return BT::NodeStatus::FAILURE;}
        if(!getInput<int>("my_outpost_blood_upperbound",my_outpost_blood_upperbound)) {RCLCPP_INFO(rclcpp::get_logger("outpost_judge"),"my outpost upper");return BT::NodeStatus::FAILURE;}
        if(!getInput<int>("enemy_outpost_blood_lowerbound",enemy_outpost_blood_lowerbound)) {RCLCPP_INFO(rclcpp::get_logger("outpost_judge"),"enemy outpost lower");return BT::NodeStatus::FAILURE;}
        if(!getInput<int>("enemy_outpost_blood_upperbound",enemy_outpost_blood_upperbound)) {RCLCPP_INFO(rclcpp::get_logger("outpost_judge"),"enemy outpost upper");return BT::NodeStatus::FAILURE;}
        if(my_outpost < my_outpost_blood_lowerbound) return BT::NodeStatus::FAILURE;
        if(my_outpost > my_outpost_blood_upperbound) return BT::NodeStatus::FAILURE;
        if(enemy_outpost < enemy_outpost_blood_lowerbound) return BT::NodeStatus::FAILURE;
        if(enemy_outpost > enemy_outpost_blood_upperbound) return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::SUCCESS;
    }
}