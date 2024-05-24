#ifndef RM_SENTRY_2024_TOPIC_TRANSMIT_NODE_
#define RM_SENTRY_2024_TOPIC_TRANSMIT_NODE_
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/gimbal_data.hpp"
#include "robot_msgs/msg/referee_data.hpp"
#include "robot_msgs/msg/build_state.hpp"
#include "rm_interfaces/msg/perception.hpp"


using namespace std::chrono_literals;

namespace topic_transimit_node {
    class TopicTransmitter : public rclcpp::Node {
    public :
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_1;
        rclcpp::Publisher<robot_msgs::msg::AutoaimInfo>::SharedPtr publisher_2;
        rclcpp::Publisher<robot_msgs::msg::RobotBloodInfo>::SharedPtr publisher_3;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_4;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_5;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_6;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_7;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_8;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_9;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_spin;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_my_sentry_blood;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_enemy_sentry_blood;
        rclcpp::Publisher<robot_msgs::msg::BuildState>::SharedPtr publisher_my_outpost_blood;
        rclcpp::Publisher<robot_msgs::msg::BuildState>::SharedPtr publisher_enemy_outpost_blood;

        rclcpp::Subscription<robot_msgs::msg::GimbalData>::SharedPtr subscription_1;
        rclcpp::Subscription<robot_msgs::msg::RefereeData>::SharedPtr subscription_2;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_spin;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_1;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_2;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_3;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_4;

        std_msgs::msg::Int32 game_time_msg;
        std_msgs::msg::Int32 self_id_msg;
        robot_msgs::msg::RobotBloodInfo robot_blood_msg;
        std_msgs::msg::Int32 referee_msg;
        std_msgs::msg::Int32 vitual_mode_msg;
        std_msgs::msg::Int32 bullet_number_msg;

        int spin_flag;
        
        TopicTransmitter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("TopicTransmitter"),
                  timer_(this->create_wall_timer(std::chrono::milliseconds(5), [this] { behaviour_publish(); })) {
                  publisher_1 = this->create_publisher<std_msgs::msg::Int32>("/game_time", 10);
                  publisher_2 = this->create_publisher<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10);
                  publisher_3 = this->create_publisher<robot_msgs::msg::RobotBloodInfo>("/robot_blood_info", 10);
                  publisher_4 = this->create_publisher<std_msgs::msg::Int32>("/vitual_mode", 10);
                  publisher_5 = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_Vehicle", 10);
                  publisher_6 = this->create_publisher<std_msgs::msg::Int32>("/self_id", 10);
                  publisher_7 = this->create_publisher<std_msgs::msg::Int32>("/EC2DSbase", 10);
                  publisher_8 = this->create_publisher<std_msgs::msg::Int32>("/bullet_number", 10);
                  publisher_9 = this->create_publisher<std_msgs::msg::Int32>("/ECBaseAngle",10);
                  publisher_spin = this->create_publisher<std_msgs::msg::Int32>("/decision2ECbasespin",10);
                  publisher_my_outpost_blood = this->create_publisher<robot_msgs::msg::BuildState>("my_outpost_blood", 10);
                  publisher_enemy_outpost_blood = this->create_publisher<robot_msgs::msg::BuildState>("enemy_outpost_blood", 10);
                  publisher_my_sentry_blood = this->create_publisher<std_msgs::msg::Int32>("my_sentry_blood", 10);
                  publisher_enemy_sentry_blood = this->create_publisher<std_msgs::msg::Int32>("enemy_sentry_blood", 10);

                  subscription_perception_1 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_1/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_1, this, std::placeholders::_1));
                  subscription_perception_2 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_2/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_2, this, std::placeholders::_1));
                  subscription_perception_3 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_3/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_3, this, std::placeholders::_1));
                  subscription_perception_4 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_4/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_4, this, std::placeholders::_1));
                  
                
                  //subscription_enemy_pos = gimbal_choose_enemy_node->create_subscription<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10, std::bind(&GimbalChooseEnemyNode::message_callback_enemy_pos, this, std::placeholders::_1));

                  subscription_1 = this->create_subscription<robot_msgs::msg::GimbalData>("/easy_robot_commands/offset_data",10,std::bind(&TopicTransmitter::subscription_Gimbal_Data,this,std::placeholders::_1));
                  subscription_2 = this->create_subscription<robot_msgs::msg::RefereeData>("/easy_robot_commands/referee_data_for_decision",10,std::bind(&TopicTransmitter::subscription_Referee_Data,this,std::placeholders::_1));
                  subscription_spin = this->create_subscription<std_msgs::msg::Int32>("/decision2ECbasespin",10,std::bind(&TopicTransmitter::subscription_Spin_Data,this,std::placeholders::_1));
                  spin_flag = -1;
        }

        std::map<int,geometry_msgs::msg::Point> robot_pos_map;

        void subscription_Perception_1(const rm_interfaces::msg::Perception &msg){
            // for(auto element:msg.enemys){
            //     robot_pos_map[element.id] = element;
            // }
            return;
        }

        void subscription_Perception_2(const rm_interfaces::msg::Perception &msg){
            // for(auto element:msg.enemys){
            //     robot_pos_map[element.id] = element;
            // }
            return;
        }

        void subscription_Perception_3(const rm_interfaces::msg::Perception &msg){
            // for(auto element:msg.enemys){
            //     robot_pos_map[element.id] = element;
            // }
            return;
        }

        void subscription_Perception_4(const rm_interfaces::msg::Perception &msg){
            // for(auto element:msg.enemys){
            //     robot_pos_map[element.id] = element;
            // }
            return;
        }

        void subscription_Spin_Data(const std_msgs::msg::Int32 &msg){
            spin_flag = msg.data;
        }

        void subscription_Gimbal_Data(const robot_msgs::msg::GimbalData &msg){
            referee_msg.data = msg.offset;
            vitual_mode_msg.data = msg.virtual_mode;
            return;
        }

        void subscription_Referee_Data(const robot_msgs::msg::RefereeData &msg){
            game_time_msg.data = (int)msg.game_time;
            bullet_number_msg.data = (int)msg.allow_bullet;
            self_id_msg.data = (int)msg.robot_id;
            robot_blood_msg.data = msg.data;
            return;
        }
        
        void behaviour_publish() {


            int my_id = self_id_msg.data;
            std_msgs::msg::Int32 my_blood,enemy_blood;
            for(int i = 0; i < robot_blood_msg.data.size(); ++i){
                robot_msgs::msg::BuildState BuildState;
                BuildState.id = robot_blood_msg.data[i].id;
                BuildState.blood = robot_blood_msg.data[i].blood;
                if(BuildState.id/100 == my_id/100 && BuildState.id%100 == 8){
                    publisher_my_outpost_blood->publish(BuildState);
                }
                if(BuildState.id/100 != my_id/100 && BuildState.id%100 == 8){
                    publisher_enemy_outpost_blood->publish(BuildState);
                }
            }
            publisher_1->publish(game_time_msg);
            publisher_3->publish(robot_blood_msg);
            publisher_4->publish(vitual_mode_msg);
            publisher_6->publish(self_id_msg);
            publisher_7->publish(referee_msg);
            publisher_8->publish(bullet_number_msg);
            if(game_time_msg.data == 0){
                std_msgs::msg::Int32 spin_msg;spin_msg.data = 6;
                publisher_spin->publish(spin_msg);
                return;
            }
            if(spin_flag != 6){
                spin_flag = 0;
                std_msgs::msg::Int32 spin_msg;spin_msg.data = spin_flag;
                publisher_spin->publish(spin_msg);
            }
            spin_flag = -1;
            return;
        }

    private :
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(topic_transimit_node::TopicTransmitter)

#endif