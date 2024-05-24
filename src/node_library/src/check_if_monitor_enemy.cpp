#include "node_library/check_if_monitor_enemy.hpp"

namespace BehaviorTree{
    CheckIfMonitorEnemy::CheckIfMonitorEnemy(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "check_if_monitor_enemy";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_blood = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info",10,[this](const robot_msgs::msg::RobotBloodInfo::SharedPtr msg){
                        for(auto i:msg->data){
                            if(i.blood>0 || robot_list.find(i.id)!=robot_list.end())
                            robot_list[i.id] = i.blood;
                        }
                    });
                    subscription_id = node1->create_subscription<std_msgs::msg::Int32>("/self_id",10,[this](const std_msgs::msg::Int32::SharedPtr msg){
                        self_id = msg->data;
                    });
                    publish_flag = 0;
                    self_id = -1;
                }
    BT::NodeStatus CheckIfMonitorEnemy::tick()
    {
        rclcpp::spin_some(node1);
        int true_or_false = 1;
        double delay_time = 1;
        getInput<int>("true_or_false",true_or_false);
        auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
        if(true_or_false != 0) std::swap(FAILURE,SUCCESS);
        int flag = -1;
        for(auto i:robot_list){
            if(i.first/100 != self_id/100){
                if(flag < 0)
                flag = 2;
                if(i.first%100 == 7 && i.second > 0){
                    flag = -1;
                    break;
                }
                if(i.first%100 == 1 && i.second<=0){
                    flag |= 1;
                }
                if(i.second > 0){
                    flag ^= (flag&2);
                }
            }
        }
        if(flag>0) return SUCCESS;
        return FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::CheckIfMonitorEnemy>("NavUNode");
// }