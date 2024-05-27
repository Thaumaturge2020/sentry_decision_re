#include "node_library/gimbal_choose_building_node.hpp"

namespace BehaviorTree
{
    GimbalChooseBuildingNode::GimbalChooseBuildingNode(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
    {
        const auto map_data = toml::parse(ROOT "config/graph_information.toml");
        const auto battle_data = toml::parse(ROOT "config/battle_information.toml");
        rclcpp::Time ti_now = rclcpp::Clock().now();

        std::stringstream ss;
        ss << "gimbal_choose_building_node";
        gimbal_choose_building_node = rclcpp::Node::make_shared(ss.str().c_str());

        game_time = 0;
        buff_status = 0;

        subscription_base_angle = gimbal_choose_building_node->create_subscription<std_msgs::msg::Float64>("EC2DS/base", 10, std::bind(&GimbalChooseBuildingNode::message_callback_base_angle, this, std::placeholders::_1));
        subscription_cam_angle = gimbal_choose_building_node->create_subscription<std_msgs::msg::Float64>("EC2DS/cam", 10, std::bind(&GimbalChooseBuildingNode::message_callback_cam_angle, this, std::placeholders::_1));
        subscription_base2cam_angle = gimbal_choose_building_node->create_subscription<std_msgs::msg::Float64>("EC2DS/base2cam", 10, std::bind(&GimbalChooseBuildingNode::message_callback_base2cam_angle, this, std::placeholders::_1));
        subscription_autoaim_able = gimbal_choose_building_node->create_subscription<std_msgs::msg::Int32>("/vitual_mode", 10, std::bind(&GimbalChooseBuildingNode::message_callback_autoaim_able, this, std::placeholders::_1));
        subscription_enemy_pos = gimbal_choose_building_node->create_subscription<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10, std::bind(&GimbalChooseBuildingNode::message_callback_enemy_pos, this, std::placeholders::_1));
        subscription_my_pos = gimbal_choose_building_node->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&GimbalChooseBuildingNode::message_callback_my_pos, this, std::placeholders::_1));
        subscription_game_time = gimbal_choose_building_node->create_subscription<std_msgs::msg::Int32>("/game_time",10,[this](const std_msgs::msg::Int32 &msg){
            game_time = msg.data;
            if(game_time < 210)
            buff_status = 1;
        });
        publisher_enemy_info = gimbal_choose_building_node->create_publisher<robot_msgs::msg::CamCommand>("/decision2transmit", 10);
        building_pos = toml::find<std::vector<std::vector< double > > >(battle_data,"building_pos");
        self_start_point = toml::find<std::pair<double,double> >(battle_data,"self_position");

        base2cam_angle = 0;
        base_angle = 0;
        cam_angle = 0;
    }

    void GimbalChooseBuildingNode::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        tf2::Quaternion imu_quat(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(imu_quat);
        m.getRPY(roll, pitch, yaw);
        base_angle = yaw;
        My_pos.first = msg.pose.pose.position.x + self_start_point.first;
        My_pos.second = msg.pose.pose.position.y + self_start_point.second;
        return;
    }

    void GimbalChooseBuildingNode::message_callback_base_angle(const std_msgs::msg::Float64 &msg)
    {
        base_angle = msg.data;
        return;
    }

    void GimbalChooseBuildingNode::message_callback_cam_angle(const std_msgs::msg::Float64 &msg)
    {
        cam_angle = msg.data;
        return;
    }

    void GimbalChooseBuildingNode::message_callback_base2cam_angle(const std_msgs::msg::Float64 &msg)
    {
        base2cam_angle = msg.data;
        return;
    }

    void GimbalChooseBuildingNode::message_callback_autoaim_able(const std_msgs::msg::Int32 &msg)
    {
        autoaim_able = msg.data;
        return;
    }

    void GimbalChooseBuildingNode::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        enemy_pos = msg.data;
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseBuildingNode"), "Successfully callback.");
    }

    BT::NodeStatus GimbalChooseBuildingNode::tick()
    {
        // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseBuildingNode"), "I'm ticked");
        rclcpp::spin_some(gimbal_choose_building_node);
        //
        int building_id;
        if (!getInput<int>("id", building_id))
        {
            // RCLCPP_INFO(rclcpp::get_logger("GimbalChooseBuildingNode"), "No Response");
            return BT::NodeStatus::FAILURE;
        }

        double my_pos_angle = 0;

        if(building_id != 1 && building_id != 101 && building_id != 8 && building_id != 9 && building_id!=108 && building_id!=109) {
            RCLCPP_INFO(rclcpp::get_logger("navigation_building_node"), "Wrong Input");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Point pos;
        
        if(building_id%100 != 1){
            pos.x = building_pos[(building_id%100 == 8)+building_id/100*3][0];
            pos.y = building_pos[(building_id%100 == 8)+building_id/100*3][1];
            pos.z = 0;
        }
        else{
            pos.x = building_pos[2 + building_id/100 * 3][0];
            pos.y = building_pos[2 + building_id/100 * 3][0];
            pos.z = 0;
        }

        my_pos_angle = atan2(pos.y - My_pos.second,pos.x - My_pos.first);

        double azimuth = my_pos_angle + cam_angle - base_angle - base2cam_angle;
        robot_msgs::msg::CamCommand pub_msg;
        pub_msg.autoaim_mode = 2;
        if(building_id%100 == 1)
        pub_msg.autoaim_mode = buff_status ? 5:4;

        unsigned char element = building_id%100;
        std::vector<unsigned char> empty_arr;
        pub_msg.priority_type_arr = empty_arr;
        pub_msg.priority_level_arr = empty_arr;
        pub_msg.priority_type_arr.resize(8);
        pub_msg.priority_level_arr.resize(8);
        for(int i=0;i<8;++i) pub_msg.priority_type_arr[i] = 1;
        
        if(building_id%100 != 1)
        pub_msg.priority_type_arr[decision_utils::id_mapping::autoaim2priority(building_id%100)] = 0;
        
        for(int i=0;i<8;++i) pub_msg.priority_level_arr[i] = 1;
        
        if(building_id%100 != 1)
        pub_msg.priority_level_arr[decision_utils::id_mapping::autoaim2priority(building_id%100)] = 8;

        // pub_msg.enemy_id = building_id;
        pub_msg.yaw = azimuth;
        pub_msg.pitch_mode = 1;
        pub_msg.autoshoot_rate = 10;
        // �����λ��
        publisher_enemy_info->publish(pub_msg);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<BehaviorTree::GimbalChooseBuildingNode>("GimbalChooseBuildingNode");
// }
