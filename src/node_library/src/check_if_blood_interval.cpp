#include "node_library/check_if_blood_interval.hpp"

namespace BehaviorTree{
    CheckIfBloodInterval::CheckIfBloodInterval(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();

                    std::stringstream ss;
                    ss << "check_if_blood_interval";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    getInput<int>("minimum_blood",minimum_blood);
                    getInput<int>("maximum_blood",maximum_blood);
                    subscription_blood = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info",10,[this](const robot_msgs::msg::RobotBloodInfo::SharedPtr msg){
                        for(auto i:msg->data){
                            if(i.id == target_id){
                                if(i.blood < minimum_blood || i.blood > maximum_blood)  publish_flag = 0;
                                else                                                    publish_flag = 1;
                                break;
                            }
                        }
                    });
                    subscription_id = node1->create_subscription<std_msgs::msg::Int32>("/self_id",10,[this](const std_msgs::msg::Int32::SharedPtr msg){
                        self_id = msg->data;
                        getInput<int>("target_id",target_id);
                        if(self_id/100 == 1){
                            target_id = (target_id/100)?target_id-100:target_id+100;
                        }
                    });
                    publish_flag = 0;
                    self_id = 7;
                    target_id = -1;
                }

    BT::NodeStatus CheckIfBloodInterval::tick()
    {
        rclcpp::spin_some(node1);
        int true_or_false = 1;
        getInput<int>("true_or_false",true_or_false);
        auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
        if(true_or_false != 0) std::swap(FAILURE,SUCCESS);
        if(publish_flag == 1)   return SUCCESS;
        else                    return FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::CheckIfBloodInterval>("NavUNode");
// }