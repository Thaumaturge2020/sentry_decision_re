#include "node_library/judgeoutpost.hpp"

namespace BehaviorTree{
    
    judgeoutpost::judgeoutpost(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    limit_distance=100.0;//初始化限制距离
                    //outpost_posfu赋值前少站
                    // outpost_pos = toml::find
                    std::stringstream ss;
                    ss << "judgeoutpost";
                    judgeoutp = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_judpos_pos = judgeoutp->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&judgeoutpost::message_callback_judpos_pos,this,std::placeholders::_1));
                }
    void judgeoutpost::message_callback_judpos_pos(const nav_msgs::msg::Odometry &msg){
        judpos_pos=msg.pose.pose.position;
    }

    BT::NodeStatus judgeoutpost::tick(){
        if((judpos_pos.x-outpost_pos.x)*(judpos_pos.x-outpost_pos.x)+(judpos_pos.y-outpost_pos.y)*(judpos_pos.y-outpost_pos.y)+(judpos_pos.z-outpost_pos.z)*(judpos_pos.z-outpost_pos.z)>limit_distance)
        {
            
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    
            
    
}