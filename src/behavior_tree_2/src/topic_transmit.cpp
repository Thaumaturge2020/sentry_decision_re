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
#include "robot_msgs/msg/cam_command.hpp"
#include "robot_msgs/msg/decision_points.hpp"
#include "robot_msgs/msg/chassis_info.hpp"
#include "rm_interfaces/msg/perception.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robot_msgs/msg/walk_cmd.hpp"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "toml.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "behavior_tree_2/topic_transmit.hpp"


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
        rclcpp::Publisher<robot_msgs::msg::CamCommand>::SharedPtr publisher_10;
        rclcpp::Publisher<robot_msgs::msg::DecisionPoints>::SharedPtr publisher_11;
        rclcpp::Publisher<robot_msgs::msg::WalkCmd>::SharedPtr publisher_12;

        rclcpp::Subscription<robot_msgs::msg::GimbalData>::SharedPtr subscription_1;
        rclcpp::Subscription<robot_msgs::msg::RefereeData>::SharedPtr subscription_2;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_spin;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_1;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_2;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_3;
        rclcpp::Subscription<rm_interfaces::msg::Perception>::SharedPtr subscription_perception_4;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_pose_stamped;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry;
        rclcpp::Subscription<robot_msgs::msg::WalkCmd>::SharedPtr subscription_goal;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_vel;

        rclcpp::Subscription<robot_msgs::msg::ChassisInfo>::SharedPtr subscription_chassis_info;


        rclcpp::Subscription<robot_msgs::msg::CamCommand>::SharedPtr subscription_enemy_pos;

        std_msgs::msg::Int32 game_time_msg;
        std_msgs::msg::Int32 self_id_msg;
        robot_msgs::msg::RobotBloodInfo robot_blood_msg;
        std_msgs::msg::Int32 referee_msg;
        std_msgs::msg::Int32 vitual_mode_msg;
        std_msgs::msg::Int32 bullet_number_msg;

        int spin_flag;

        int ally_sentry_invicinble,enemy_sentry_invicinble;

        double sentry_pub_omiga;

        geometry_msgs::msg::Point goal_position,now_position,next_position;

        Eigen::Matrix3d trans;

        double sentry_vel;

        
        
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
                  publisher_10 = this->create_publisher<robot_msgs::msg::CamCommand>("/decision2autoaim",10);
                  publisher_11 = this->create_publisher<robot_msgs::msg::DecisionPoints>("/easy_robot_commands/decision_points",10);
                  publisher_12 = this->create_publisher<robot_msgs::msg::WalkCmd>("/decision2pathplan",10);
                  publisher_spin = this->create_publisher<std_msgs::msg::Int32>("/decision2ECbasespin",10);
                  publisher_my_outpost_blood = this->create_publisher<robot_msgs::msg::BuildState>("my_outpost_blood", 10);
                  publisher_enemy_outpost_blood = this->create_publisher<robot_msgs::msg::BuildState>("enemy_outpost_blood", 10);
                  publisher_my_sentry_blood = this->create_publisher<std_msgs::msg::Int32>("my_sentry_blood", 10);
                  publisher_enemy_sentry_blood = this->create_publisher<std_msgs::msg::Int32>("enemy_sentry_blood", 10);

                  sentry_vel = 0;
                  sentry_pub_omiga = 0;

                  const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                  std::array<std::array<double,3>,3> trans_matrix = toml::find<std::array<std::array<double,3>,3> > (toml_file,"trans_matrix");
                  for(int i=0;i<3;++i) for(int j=0;j<3;++j) trans(i,j) = trans_matrix[i][j];

                  trans = trans.inverse().eval();

                  subscription_perception_1 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_1/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_1, this, std::placeholders::_1));
                  subscription_perception_2 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_2/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_2, this, std::placeholders::_1));
                  subscription_perception_3 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_3/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_3, this, std::placeholders::_1));
                  subscription_perception_4 = this->create_subscription<rm_interfaces::msg::Perception>("/detection_4/detection_armor_enemy", 10, std::bind(&TopicTransmitter::subscription_Perception_4, this, std::placeholders::_1));
                  subscription_odometry = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,[this](const nav_msgs::msg::Odometry &msg){
                    now_position = msg.pose.pose.position;
                  });
                
                  subscription_enemy_pos = this->create_subscription<robot_msgs::msg::CamCommand>("/decision2transmit", 10, [this](const robot_msgs::msg::CamCommand &rec_msg){
                    robot_msgs::msg::CamCommand msg = rec_msg;
                    if(enemy_sentry_invicinble){
                        msg.priority_type_arr[5] = 2;
                    }
                    if(sentry_vel > 1000){
                        msg.priority_type_arr[6] = 2;
                        msg.priority_type_arr[7] = 2;
                    }
                    publisher_10->publish(msg);
                  });

                  subscription_pose_stamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,[this](const geometry_msgs::msg::PoseStamped &msg){
                    next_position = msg.pose.position;
                  });

                  subscription_goal = this->create_subscription<robot_msgs::msg::WalkCmd>("/decision2transplan",10,[this](const robot_msgs::msg::WalkCmd &msg){
                    goal_position = msg.pos;
                    robot_msgs::msg::DecisionPoints pub_msg;
                    robot_msgs::msg::WalkCmd walk_msg = msg;
                    pub_msg.intention = 3;
                    Eigen::Vector3d now_position_vec = trans*Eigen::Vector3d(now_position.x,now_position.y,now_position.z);
                    Eigen::Vector3d next_position_vec = trans*Eigen::Vector3d(next_position.x,next_position.y,next_position.z);
                    Eigen::Vector3d goal_position_vec = trans*Eigen::Vector3d(goal_position.x,goal_position.y,goal_position.z);
                    geometry_msgs::msg::Point now_position_2,next_position_2,goal_position_2;
                    now_position_2.x = now_position_vec[0],now_position_2.y = now_position_vec[1],now_position_2.z = now_position_vec[2];
                    next_position_2.x = next_position_vec[0],next_position_2.y = next_position_vec[1],next_position_2.z = next_position_vec[2];
                    goal_position_2.x = goal_position_vec[0],goal_position_2.y = goal_position_vec[1],goal_position_2.z = goal_position_vec[2];
                    
                    pub_msg.start = now_position_2;
                    pub_msg.point1 = next_position_2;
                    pub_msg.point2 = goal_position_2;
                    publisher_11->publish(pub_msg);
                    
                    if(ally_sentry_invicinble)
                    walk_msg.radium = 0;
                    else
                    walk_msg.radium = sentry_pub_omiga;
                    publisher_12->publish(walk_msg);
                  });

                  subscription_1 = this->create_subscription<robot_msgs::msg::GimbalData>("/easy_robot_commands/offset_data",10,std::bind(&TopicTransmitter::subscription_Gimbal_Data,this,std::placeholders::_1));
                  subscription_2 = this->create_subscription<robot_msgs::msg::RefereeData>("/easy_robot_commands/referee_data_for_decision",10,std::bind(&TopicTransmitter::subscription_Referee_Data,this,std::placeholders::_1));
                  subscription_spin = this->create_subscription<std_msgs::msg::Int32>("/decision2ECbasespin",10,std::bind(&TopicTransmitter::subscription_Spin_Data,this,std::placeholders::_1));

                  subscription_vel = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/cmd_vel",10,[this](const geometry_msgs::msg::WrenchStamped &msg){
                    //sentry_vel = sqrt((msg.wrench.force.x*msg.wrench.force.x)+(msg.wrench.force.y*msg.wrench.force.y));
                    if(game_time_msg.data > 0 && game_time_msg.data <= 420)
                    sentry_pub_omiga = 540 / (sentry_vel/100 + 1);
                    else
                    sentry_pub_omiga = 0;
                  });

                  subscription_chassis_info = this->create_subscription<robot_msgs::msg::ChassisInfo>("/easy_robot_commands/chassis_info",10,[this](const robot_msgs::msg::ChassisInfo &msg){
                    double  vel_x = msg.vx,
                            vel_y = msg.vy;
                    sentry_vel = sqrt(vel_x*vel_x+vel_y*vel_y);
                  });
                  
                  spin_flag = -1;
                  enemy_sentry_invicinble = 1;
                  ally_sentry_invicinble = 1;
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
            for(int i=0,lim = robot_blood_msg.data.size();i<lim;++i){
                if(robot_blood_msg.data[i].id % 100 == 8){
                    if(robot_blood_msg.data[i].id / 100 == self_id_msg.data / 100){
                        if(robot_blood_msg.data[i].blood < 1500)
                        ally_sentry_invicinble = 0;
                    }
                    else{
                        if(robot_blood_msg.data[i].blood < 1500)
                        enemy_sentry_invicinble = 0;
                    }
                }
            }
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