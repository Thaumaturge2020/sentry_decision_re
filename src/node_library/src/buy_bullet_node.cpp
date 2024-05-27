#include "node_library/buy_bullet_node.hpp"

namespace BehaviorTree{

    BuyBulletNode::BuyBulletNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "BuyBulletNode";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    publisher = node1->create_publisher<robot_msgs::msg::SentryRefereeDecision>("/easy_robot_commands/sentry_referee_decision",10);

                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    #define pdd std::pair<double,double>
                    bias_position = toml::find<pdd>(toml_file,"self_position");
                    #undef pdd
                    
                    subscription_1 = node1->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,[this](const nav_msgs::msg::Odometry &msg){
                        self_position.first = msg.pose.pose.position.x + bias_position.first;
                        self_position.second = msg.pose.pose.position.y + bias_position.second;
                    });
                    
                    start_reach_Time = rclcpp::Clock().now();

                    int area_id = 0;getInput<int>("area_id",area_id);
                    std::vector<std::vector<std::pair<double,double> > > navigate_area = toml::find< std::vector<std::vector<std::pair<double,double> > > > (toml_file,"navigation_position");
                    navigation_point.x = navigate_area[area_id][0].first;
                    navigation_point.y = navigate_area[area_id][0].second;
                    navigation_point.z = 0;
                }

    BT::NodeStatus BuyBulletNode::tick()
    {
        
        // RCLCPP_INFO(rclcpp::get_logger("BuyBulletNode"),"I'm ticked");
        rclcpp::spin_some(node1);
        int given_id,bullet_buy_num;

        if(Eigen::Vector3d(self_position.first - navigation_point.x,self_position.second - navigation_point.y,0).norm() < 1){
            able_buy = 1;
            start_reach_Time = rclcpp::Clock().now();
        }

        // RCLCPP_INFO(rclcpp::get_logger("self_position"),"%d %lf %lf %lf %lf %lf",able_buy,self_position.first,navigation_point.x,self_position.second,navigation_point.y,Eigen::Vector3d(self_position.first - navigation_point.x,self_position.second - navigation_point.y,0).norm());
        
        if(able_buy && (rclcpp::Clock().now() - start_reach_Time).seconds() >= 2.5){
            if(getInput<int>("bullet_buy_num",bullet_buy_num)){
                able_buy = 0;
                robot_msgs::msg::SentryRefereeDecision msg;
                msg.bullet_num_to_buy = bullet_buy_num;
                publisher->publish(msg);
                setOutput<int>("reset_aerial_type",-1);
            }
        }
        return BT::NodeStatus::SUCCESS;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::BuyBulletNode>("BuildAttackNode");
// }