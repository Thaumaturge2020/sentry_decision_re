#include "node_library/defence_building_node.hpp"

namespace BehaviorTree
{
    DefenceBuildingNode::DefenceBuildingNode(const std::string &name, const BT::NodeConfig &config) :
            BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();

        std::stringstream ss;
        ss << "defence_building_node";
        node1 = rclcpp::Node::make_shared(ss.str().c_str());
        subscription_build_state = node1->create_subscription<robot_msgs::msg::RobotBloodInfo>("robot_blood_info", 10, std::bind(&DefenceBuildingNode::message_callback_build_state, this, std::placeholders::_1));
        subscription_build_id = node1->create_subscription<std_msgs::msg::Int32>("self_id", 10, std::bind(&DefenceBuildingNode::message_callback_self_id, this, std::placeholders::_1));
        const auto toml_file = toml::parse(ROOT "/config/battle_information.toml");
        build_blood_threshold = toml::find<int>(toml_file,"build_blood_threshold");
        self_id = -1;
    }

    void DefenceBuildingNode::message_callback_build_state(const robot_msgs::msg::RobotBloodInfo &msg)
    {
        build_state_array = msg.data;
        return;
    }

    void DefenceBuildingNode::message_callback_self_id(const std_msgs::msg::Int32 &msg)
    {
        self_id = msg.data;
        return;
    }

    BT::NodeStatus DefenceBuildingNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "I'm ticked");
        rclcpp::spin_some(node1);
        size_t array_size = build_state_array.size();
        int building_id,building_type,build_blood_threshold;
        if (!getInput<int>("building_id", building_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }
        //0 for teammate,1 for enemy.
        if (!getInput<int>("building_type", building_type))
        {
            // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<int>("building_blood_threshold", build_blood_threshold))
        {
            // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "No Response");
            return BT::NodeStatus::FAILURE;
        }
        if (self_id < 0)
        {
            return BT::NodeStatus::FAILURE;
        }

        // RCLCPP_INFO(rclcpp::get_logger("defence"),"this node");

        int now_id = building_id + (self_id / 100 == 0 ? building_type : (building_type^1)) * 100;

        // RCLCPP_INFO(rclcpp::get_logger("defence"),"now_id: %d",now_id);

        for (int i = 0; i < array_size; ++i)
        {
            // RCLCPP_INFO(rclcpp::get_logger("defence"),"array_id: %d",build_state_array[i].id);
            if (build_state_array[i].id == now_id)
            {
                if(build_state_array[i].blood >= build_blood_threshold){
                    // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "building no need defending");
                    return BT::NodeStatus::SUCCESS;
                }
                
                // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "building Found");
                return BT::NodeStatus::FAILURE;
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("defence_building_node"), "Not Found");
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::DefenceBuildingNode>("DefenceBuildingNode");
// }