#include "node_library/on_fire_node.hpp"

namespace BehaviorTree{
    OnFireNode::OnFireNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "on_fire_node";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_fire_pos = node1->create_subscription<std_msgs::msg::Int32>("vitual_mode",10,[this](const std_msgs::msg::Int32::SharedPtr msg){
                        gimbal_state = msg->data;
                    });
                    publisher_spin = node1->create_publisher<std_msgs::msg::Int32>("/decision2ECbasespin",10);                    
                    gimbal_state = 0;
                }
    BT::NodeStatus OnFireNode::tick()
    {
        rclcpp::spin_some(node1);
        int true_or_false = 1;
        getInput<int>("true_or_false",true_or_false);
        auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
        if(true_or_false != 0) std::swap(FAILURE,SUCCESS);
        BT::NodeStatus now_status = SUCCESS;
        if(gimbal_state != 0){
            now_status = SUCCESS;
        }
        else{
            //RCLCPP_INFO(rclcpp::get_logger("thss...."),"%d",)
            now_status = FAILURE;
        }
        std_msgs::msg::Int32 pub_msg;
        pub_msg.data = 6;
        if(now_status == SUCCESS){
            // publisher_spin->publish(pub_msg);
            ;
        }
        return now_status;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::OnFireNode>("NavUNode");
// }