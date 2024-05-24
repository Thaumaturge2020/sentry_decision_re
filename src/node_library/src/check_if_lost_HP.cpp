#include "node_library/check_if_lost_HP.hpp"

namespace BehaviorTree{
    CheckIfLostHP::CheckIfLostHP(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "check_if_lost_HP";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_blood = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info",10,[this](const robot_msgs::msg::RobotBloodInfo::SharedPtr msg){
                        for(auto i:msg->data){
                            if(i.id == self_id){
                                if(now_HP > i.blood){
                                    publish_flag = 1;
                                    now_lose_HP_time = rclcpp::Clock().now();
                                }
                                now_HP = i.blood;
                            }
                        }
                    });
                    subscription_id = node1->create_subscription<std_msgs::msg::Int32>("/self_id",10,[this](const std_msgs::msg::Int32::SharedPtr msg){
                        self_id = msg->data;
                    });
                    publish_flag = 0;
                    self_id = -1;
                    now_HP = 600;
                    now_lose_HP_time = rclcpp::Clock().now();
                }
    BT::NodeStatus CheckIfLostHP::tick()
    {
        rclcpp::spin_some(node1);
        int true_or_false = 1;
        double delay_time = 1;
        getInput<int>("true_or_false",true_or_false);
        getInput<double>("delay_time",delay_time);
        auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
        if(true_or_false != 0) std::swap(FAILURE,SUCCESS);
        if(publish_flag == 1 && (rclcpp::Clock().now() - now_lose_HP_time).seconds() < delay_time)      return SUCCESS;
        else                                                                                            return FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::CheckIfLostHP>("NavUNode");
// }