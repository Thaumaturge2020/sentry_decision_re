#include "node_library/get_opblood_node.hpp"

namespace BehaviorTree{
    GetOpbloodNode::GetOpbloodNode(const std::string &name, const BT::NodeConfig &config) :
            BT::SyncActionNode(name, config)
            {
                enemy_opblood = -1;
                my_opblood = -1;
                rclcpp::Time ti_now = rclcpp::Clock().now();
                std::stringstream ss;
                ss << "get_opblood_node";
                node1 = rclcpp::Node::make_shared(ss.str().c_str());
                subscription_enemy_op_info = node1 -> create_subscription<robot_msgs::msg::BuildState>("enemyop_info",10,std::bind(&GetOpbloodNode::enemyop_info_recevice_callback,this,std::placeholders::_1));
                subscription_my_op_info = node1 -> create_subscription<robot_msgs::msg::BuildState>("myop_info",10,std::bind(&GetOpbloodNode::myop_info_recevice_callback,this,std::placeholders::_1));
            }

// ke yi jia ge pan duan id
    void GetOpbloodNode::enemyop_info_recevice_callback(const robot_msgs::msg::BuildState &msg)
    {
        my_opblood = msg.blood;
    }

    void GetOpbloodNode::myop_info_recevice_callback(const robot_msgs::msg::BuildState &msg)
    {
        enemy_opblood = msg.blood;
    }

    BT::NodeStatus GetOpbloodNode::tick(){
        RCLCPP_INFO(rclcpp::get_logger("lidar_attack_node"),"I'm get_opblood_node");
        rclcpp::spin_some(node1);
        setOutput<int>("enemy_outpost_blood",enemy_opblood);
        setOutput<int>("my_outpost_blood",my_opblood);
        return BT::NodeStatus::SUCCESS;
        
    }


}