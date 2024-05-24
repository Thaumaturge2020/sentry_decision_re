#include "node_library/nav_heal_node.hpp"

namespace BehaviorTree{

    nav_heal_node::nav_heal_node(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "nav_heal_node";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_blood_info = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("robot_blood_info",10,std::bind(&nav_heal_node::blood_callback,this,std::placeholders::_1));
                    subscription_robot_id = node1->create_subscription<std_msgs::msg::Int32>("self_id",10,std::bind(&nav_heal_node::robot_id_callback,this,std::placeholders::_1));
                    publisher_refresh_blood = node1->create_publisher<std_msgs::msg::Int32>("refresh_blood",10);
                    
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    auto pos = toml::find<std::pair<double,double> >(toml_file,"heal_position");
                    sentry_blood_interval = toml::find<std::vector<int> >(toml_file,"sentry_blood_interval");
                    sentry_blood_refresh_limit = toml::find<int>(toml_file,"sentry_blood_refresh_limit");
                    heal_position.x = pos.first;
                    heal_position.y = pos.second;
                    heal_position.z = 0;
                    refresh_blood_count = 0;
                    robot_id = -1;

                    getInput<int>("interval_0",sentry_blood_interval[0]);
                    getInput<int>("interval_1",sentry_blood_interval[1]);
                    initial_type = 0;
                }

    void nav_heal_node::blood_callback(const robot_msgs::msg::RobotBloodInfo &msg)
    {
        // RCLCPP_INFO(rclcpp::get_logger("nav_heal_node"),"Successfully received blood info");
        blood_info = msg;
    }
    
    void nav_heal_node::robot_id_callback(const std_msgs::msg::Int32 &msg)
    {
        robot_id = msg.data;
    }

    bool nav_heal_node::check_blood(bool type)
    {
        // RCLCPP_INFO(rclcpp::get_logger("nav_heal_node"),"refresh_blood_count:%d",refresh_blood_count);
        if(refresh_blood_count > sentry_blood_refresh_limit){
            return false;
        }
        for(auto element:blood_info.data){
            // RCLCPP_INFO(rclcpp::get_logger("nav_heal_node"),"robot_id:%d,element_id:%d,element_blood:%d",robot_id,element.id,element.blood);
            if(element.id == robot_id && element.blood < sentry_blood_interval[type])
                return true;
        }
        return false;
    }
    
    void nav_heal_node::refresh_robot_blood(int my_id)
    {
        int old_blood = 0,new_blood = 0,flag = 0;
        for(auto element:blood_info.data){
            if(element.id == my_id)
            {new_blood = element.blood;++flag;break;}
        }

        for(auto element:blood_info_last.data){
            if(element.id == my_id)
            {old_blood = element.blood;++flag;break;}
        }

        if(flag <= 1) return;

        if(new_blood > old_blood)
            refresh_blood_count += new_blood - old_blood;
    }

    BT::NodeStatus nav_heal_node::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("nav_heal_node"),"I'm ticked");
        rclcpp::spin_some(node1);

        if(robot_id == -1)
            return BT::NodeStatus::FAILURE;

        refresh_robot_blood(robot_id);

        std_msgs::msg::Int32 refresh_blood;
        refresh_blood.data = refresh_blood_count;

        publisher_refresh_blood->publish(refresh_blood);

        blood_info_last = blood_info;

        if(check_blood(initial_type))
        {
            initial_type = 1;
            setOutput<geometry_msgs::msg::Point>("heal_navigation_point",heal_position);
            return BT::NodeStatus::SUCCESS;
        }
        initial_type = 0;
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::nav_heal_node>("NavHealNode");
// }