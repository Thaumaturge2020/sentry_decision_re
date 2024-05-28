#include "node_library/time_begin.hpp"

namespace BehaviorTree{
    TimeBegin::TimeBegin(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    start_time = rclcpp::Clock().now();
                    setOutput<double>("time_during",0.0);

                    std::stringstream ss;
                    ss << "time_begin";
                    node_time_begin = rclcpp::Node::make_shared(ss.str().c_str());
                    time_begin_subscription = node_time_begin->create_subscription<std_msgs::msg::Int32>("game_time",10,std::bind(&TimeBegin::message_callback_time_begin,this,std::placeholders::_1));
                    time_stamp = 0;
                }

void TimeBegin::message_callback_time_begin(const std_msgs::msg::Int32 &msg){
    time_stamp = msg.data;
    return;
}

BT::NodeStatus BehaviorTree::TimeBegin:: tick(){
    rclcpp::spin_some(node_time_begin);
    int true_or_false = 1;
    getInput<int>("true_or_false",true_or_false);
    auto FAILURE = BT::NodeStatus::FAILURE, SUCCESS = BT::NodeStatus::SUCCESS;
    if(true_or_false == 0) std::swap(FAILURE,SUCCESS);
    //RCLCPP_INFO(rclcpp::get_logger("TimeBegin"),"time_stamp:%d , return : %d",time_stamp,FAILURE == BT::NodeStatus::FAILURE ? 0 : 1);
    if(time_stamp <= 0 || time_stamp > 420) return FAILURE;
    static int flag = 0;
    if(flag == 0){
        flag = 1;
        start_time = rclcpp::Clock().now();
    }
    now_time = rclcpp::Clock().now();
    during = now_time.seconds()-start_time.seconds();
    setOutput<double>("time_during",during);
    // RCLCPP_INFO(rclcpp::get_logger("TimeBegin"),"SUCCESS");
    return SUCCESS;
}


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::TimeBegin>("TimeBegin");
// }