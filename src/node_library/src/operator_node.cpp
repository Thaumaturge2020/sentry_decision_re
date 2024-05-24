#include "node_library/operator_node.hpp"

namespace BehaviorTree{

    OperatorNode::OperatorNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "operator_node";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());        
                    target_enemy = -1;
                    aerial_type = -1;
                    subscription_operator_cmd = node1->create_subscription<robot_msgs::msg::AerialCommands>("/easy_robot_commands/map_command",10,[this](const robot_msgs::msg::AerialCommands &msg){
                        switch(msg.cmd_keyboard){
                            //控符
                            case 'T': aerial_type = 0;break;
                            //强制重启
                            case 'Y': aerial_type = 1;break;
                            //强制定点移动
                            case 'U': aerial_type = 2;break;
                            //强制回巡逻区巡逻
                            case 'I': aerial_type = 3;break;
                            //自主买弹
                            case 'P': aerial_type = 5;break;
                            //哨兵进攻基地
                            case 'L': aerial_type = 6;break;
                            //取消选择
                            case 'Q': aerial_type = -1;break;
                        }
                        target_enemy = -1;
                        if(msg.target_robot_id){
                            aerial_type = 4;
                            target_enemy = msg.target_robot_id;
                        }
                    });
                    flag = 0;
                }

    void OperatorNode::message_callback_operator_cmd(const robot_msgs::msg::AerialCommands &msg){
        navigate_point.x = msg.target_position_x;
        navigate_point.y = msg.target_position_y;
        flag = msg.cmd_keyboard;
        return;
    }

    BT::NodeStatus OperatorNode::tick(){
        // RCLCPP_INFO(rclcpp::get_logger("operator_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        
        setOutput<int>("expected_operator_mode",aerial_type);
        if(aerial_type == -1)
        return BT::NodeStatus::FAILURE;
        if(aerial_type == 4){
            setOutput<int>("expected_enemy",decision_utils::id_mapping::referee2autoaim(target_enemy));
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::OperatorNode>("OperatorNode");
// }