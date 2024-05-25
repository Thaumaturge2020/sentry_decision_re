#include "node_library/gimbal_choose_enemy_node.hpp"

namespace BehaviorTree
{
    GimbalChooseEnemyNode::GimbalChooseEnemyNode(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {
        rclcpp::Time ti_now = rclcpp::Clock().now();

        std::stringstream ss;
        ss << "gimbal_choose_enemy_node";

        gimbal_choose_enemy_node = rclcpp::Node::make_shared(ss.str().c_str());

        subscription_autoaim_able = gimbal_choose_enemy_node->create_subscription<std_msgs::msg::Int32>("/vitual_mode", 10, std::bind(&GimbalChooseEnemyNode::message_callback_autoaim_able, this, std::placeholders::_1));
        subscription_enemy_pos = gimbal_choose_enemy_node->create_subscription<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10, std::bind(&GimbalChooseEnemyNode::message_callback_enemy_pos, this, std::placeholders::_1));
        publisher_enemy_info = gimbal_choose_enemy_node->create_publisher<robot_msgs::msg::CamCommand>("/decision2transmit", 10);
    }

    void GimbalChooseEnemyNode::message_callback_autoaim_able(const std_msgs::msg::Int32 &msg)
    {
        autoaim_able = msg.data;
        return;
    }

    void GimbalChooseEnemyNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        enemy_pos = msg.data;
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Successfully callback.");
    }

    BT::NodeStatus GimbalChooseEnemyNode::tick()
    {
        rclcpp::spin_some(gimbal_choose_enemy_node);
        int now_enemy_id;
        if (!getInput<int>("now_enemy_id", now_enemy_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "No Response");
            return BT::NodeStatus::FAILURE;
        }

        std::string priority_input_str;
        std::vector<int> priority_input_int_arr;
        getInput<std::string >("priority_input_str", priority_input_str);
        getInput<std::vector<int>>("priority_input", priority_input_int_arr);
        if(priority_input_str.size() != 0){
            const auto toml_file = toml::parse(ROOT "config/node_information.toml");
            priority_input_int_arr = toml::find<std::vector<int>>(toml_file, priority_input_str.c_str());
        }

        // convert normal id to enemy id

        now_enemy_id = (now_enemy_id - 1) % 9 + 1;

        double my_pos_angle = 0;

        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "%d", now_enemy_id);
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "%d now_enemy_id:%d",enemy_pos.size(),now_enemy_id);

        for(int i=0,size = enemy_pos.size();i<size;++i){
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "now_pos_id:%d",enemy_pos[i].id);
            if(enemy_pos[i].id == now_enemy_id){
                my_pos_angle = atan2(enemy_pos[i].pos.y,enemy_pos[i].pos.x);
                goto S;
            }
        }


        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "No Detect Enemy");

        return BT::NodeStatus::FAILURE;

        S:;
        double azimuth = my_pos_angle;
        robot_msgs::msg::CamCommand pub_msg;
        // pub_msg.enemy_id = now_enemy_id;
        pub_msg.yaw = azimuth;
        pub_msg.autoaim_mode = 1;
        pub_msg.pitch_mode = 0;
        pub_msg.autoshoot_rate = 10;

        std::string my_str;
        getInput<std::string>("priority_type", my_str);
        //  RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Into 4 %s",my_str.c_str());

        std::vector<int> priority_type_int_arr;
        priority_type_int_arr = decision_utils::string_parser::GetIntArray(my_str);
        std::vector<unsigned char> priority_type_char_arr,priority_input_char_arr;

        for(auto i:priority_type_int_arr) priority_type_char_arr.push_back((unsigned char)i);
        for(auto i:priority_input_int_arr){
            priority_input_char_arr.push_back((unsigned char)i);
        }
        if(now_enemy_id >= 0){
            priority_type_char_arr[decision_utils::id_mapping::autoaim2priority(now_enemy_id)] = 0;
            priority_input_char_arr[decision_utils::id_mapping::autoaim2priority(now_enemy_id)] = 10000;
        }
        pub_msg.priority_type_arr = priority_type_char_arr;
        pub_msg.priority_level_arr = priority_input_char_arr;

        // for(auto i:pub_msg.priority_level_arr) RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Into 5 %d",i);

        // �����λ��
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseEnemyNode"), "Azimuth: %f", azimuth);
        publisher_enemy_info->publish(pub_msg);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::GimbalChooseEnemyNode>("GimbalChooseEnemyNode");
// }
