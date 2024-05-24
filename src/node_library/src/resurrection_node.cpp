#include "node_library/resurrection_node.hpp"

namespace BehaviorTree{
    ResurrectionNode::ResurrectionNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    std::stringstream ss;
                    ss << "resurrection_node";
                    node = std::make_shared<rclcpp::Node>(ss.str().c_str());
                    self_id = -1;
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    bias_position = toml::find<std::pair<double,double> >(toml_file,"self_position");
                    blood_limit = 300;
                    recover_if = 0;
                    self_last_blood = -1;
                    subscription_self_position = node->create_subscription<nav_msgs::msg::Odometry>("Odometry_Vehicle",10,[this](const nav_msgs::msg::Odometry &msg){
                        self_position.first = msg.pose.pose.position.x + bias_position.first;
                        self_position.second = msg.pose.pose.position.y + bias_position.second;
                    });
                    subscription_id = node->create_subscription<std_msgs::msg::Int32>("self_id",10,[this](const std_msgs::msg::Int32 &msg){
                        self_id = msg.data;
                    });
                    subscription_blood = node->create_subscription<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info",10,[this](const robot_msgs::msg::RobotBloodInfo &msg){
                        if(self_id == -1) return;
                        for(auto element : msg.data)if(element.id == self_id){ // 检测当哨兵机器人的血量从0变到400时，recover_if设置为1,接下来5秒在巡逻区位置等待。
                            if(self_last_blood == -1){
                                self_last_blood = element.blood;
                                break;
                            }
                            if(element.blood - self_last_blood > blood_limit){
                                recover_start_time = rclcpp::Clock().now();
                                recover_if = 1;
                            }
                            self_last_blood = element.blood;
                        }
                    });
                    publisher = node->create_publisher<robot_msgs::msg::SentryRefereeDecision>("easy_robot_commands/sentry_referee_decision",10);
                    given_distance_limit = 3;

                    
                    int area_choose;
                    getInput<int>("area_choose",area_choose);
                    auto navigation_position = toml::find<std::vector<std::vector<std::pair<double,double> > > >(toml_file,"navigation_position");
                    
                    navigation_point_input.x = navigation_position[area_choose][0].first;
                    navigation_point_input.y = navigation_position[area_choose][0].second;
                    navigation_point_input.z = 0;
                }
    BT::NodeStatus ResurrectionNode::tick()
    {
        int my_val = 0;
        rclcpp::spin_some(node);

        // RCLCPP_INFO(rclcpp::get_logger("recover_if"),"%d",recover_if);

        #define pdd std::pair<double,double>
        if(recover_if && decision_utils::judging_point::get_distance(pdd(self_position.first,self_position.second),pdd(navigation_point_input.x,navigation_point_input.y)) < given_distance_limit){
            occupy_start_time = rclcpp::Clock().now();
            recover_if = 0;
        }
        #undef pdd
        
        robot_msgs::msg::SentryRefereeDecision msg;
        msg.decide_to_resurrected = 1;
        publisher->publish(msg);

        // RCLCPP_INFO(rclcpp::get_logger("occupy_time"),"%lf",(rclcpp::Clock().now() - occupy_start_time).seconds());

        if(recover_if || (rclcpp::Clock().now() - occupy_start_time).seconds() < 5)
        return BT::NodeStatus::SUCCESS;
        
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::ResurrectionNode>("NavUNode");
// }