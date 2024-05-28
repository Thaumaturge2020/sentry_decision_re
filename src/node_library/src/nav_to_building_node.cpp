#include "node_library/nav_to_building_node.hpp"

namespace BehaviorTree
{
    const auto map_data = toml::parse(ROOT "config/graph_information.toml");
    NavToBuildingNode::NavToBuildingNode(const std::string &name, const BT::NodeConfig &config):
        BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();
    
        std::stringstream ss;
        ss << "nav_to_building_node";
        node1 = rclcpp::Node::make_shared(ss.str().c_str());
        publisher_pos = node1->create_publisher<robot_msgs::msg::WalkCmd>("/decision2transplan", 10);
        building_pos = toml::find<std::vector<std::vector<double>>>(map_data,"building_pos");
    }
    
    BT::NodeStatus NavToBuildingNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "I'm ticked");
        int building_id;
        if (!getInput<int>("id", building_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }

        if(building_id != 8 && building_id != 9 && building_id!=108 && building_id!=109) {
            RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "Wrong Input");
            return BT::NodeStatus::FAILURE;
        }
        pos.x = building_pos[(building_id%100 == 8)+building_id/100*2][0];
        pos.y = building_pos[(building_id%100 == 8)+building_id/100*2][1];
        pos.z = 0;
        robot_msgs::msg::WalkCmd my_cmd;
        my_cmd.pos = pos;
        my_cmd.opt = 2;
        my_cmd.radium = 0.0;
        my_cmd.velocity = 3500.0;
        publisher_pos->publish(my_cmd);
        RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "I'm published,the x is %f,the y is %f,the z is %f\n",pos.x,pos.y,pos.z);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::NavToBuildingNode>("NavToBuildingNode");
// }