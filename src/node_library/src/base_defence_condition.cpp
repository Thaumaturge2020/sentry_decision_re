#include "node_library/base_defense_condition.hpp"

namespace BehaviorTree{
    BaseDefenseCondition::BaseDefenseCondition(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "base_defence_condition_node";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_blood = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info",10,[this](const robot_msgs::msg::RobotBloodInfo::SharedPtr msg){
                        for(auto i:msg->data){
                            if(i.blood>0)
                            robot_list[i.id] = i.blood;
                            if(i.blood == 0 && robot_list.find(i.id) != robot_list.end() && self_id/100==i.id/100){
                                base_flag = 0;
                            }
                        }
                        int enemy_hero_id = ((self_id/100)^1)*100 + 1;
                        if(!base_flag){
                            if(robot_list.find(enemy_hero_id)!=robot_list.end() && robot_list[enemy_hero_id] != 0)  publish_flag = 0;
                            else                                                                                    publish_flag = 1;
                        }
                    });
                    subscription_id = node1->create_subscription<std_msgs::msg::Int32>("/self_id",10,[this](const std_msgs::msg::Int32::SharedPtr msg){
                        self_id = msg->data;
                    });
                    base_flag = publish_flag = 1;
                    self_id = 7;
                }
    BT::NodeStatus BaseDefenseCondition::tick()
    {
        rclcpp::spin_some(node1);
        int true_or_false = 1;
        // RCLCPP_INFO(rclcpp::get_logger("BaseDefence"),"%d %d",base_flag,self_id);
        getInput<int>("true_or_false",true_or_false);
        auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
        if(true_or_false != 0) std::swap(FAILURE,SUCCESS);
        if(publish_flag == 1)   return SUCCESS;
        else                    return FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::BaseDefenseCondition>("NavUNode");
// }