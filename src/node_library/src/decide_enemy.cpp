#include "node_library/decide_enemy.hpp"
#include "decision_utils/judging_point.hpp"

namespace BehaviorTree{

    DecideEnemy::DecideEnemy(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "decide_enemy" << std::chrono::system_clock::now().time_since_epoch().count();
                    node = rclcpp::Node::make_shared(ss.str().c_str());
                    subscription_enemy_pos = node->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&DecideEnemy::message_callback_enemy_pos,this,std::placeholders::_1));
                    subscription_enemy_blood = node->create_subscription<robot_msgs::msg::RobotBloodInfo>("robot_blood_info",10,std::bind(&DecideEnemy::message_callback_enemy_blood,this,std::placeholders::_1));
                    subscription_radar_info = node->create_subscription<robot_msgs::msg::AutoaimInfo>("radar_info",10,std::bind(&DecideEnemy::message_callback_radar_independent,this,std::placeholders::_1));
                    subscription_my_pos = node->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&DecideEnemy::message_callback_my_pos,this,std::placeholders::_1));
                    subscription_self_id = node->create_subscription<std_msgs::msg::Int32>("/self_id",10,std::bind(&DecideEnemy::message_callback_self_id,this,std::placeholders::_1));
                    subscription_base_angle = node->create_subscription<std_msgs::msg::Float64>("/ECBaseAngle",10,std::bind(&DecideEnemy::message_callback_base_angle,this,std::placeholders::_1));
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    const auto toml_battle_file = toml::parse(ROOT "config/graph_information.toml");
                    self_start_point = toml::find<std::pair<double,double> >(toml_file,"self_position");

                    robot_map_connectivity = toml::find<std::vector<std::vector<int> > >(toml_battle_file,"generate_graph_edge");
                    robot_map_field = toml::find<std::vector<std::vector<std::pair<double,double> > > >(toml_battle_file,"generate_poly");
                    robot_centre_point = toml::find<std::vector< std::pair<double,double> > >(toml_battle_file,"generate_centre_point");
                    robot_graph_height = toml::find<std::vector< double > >(toml_battle_file,"generate_graph_height");

                    

                    std::vector<double> empty;

                    robot_map_distance.clear();
                    
                    int lim = robot_map_field.size();
                    for(int i=0;i<lim;++i){
                        robot_map_distance.push_back(empty);
                        robot_map_distance[i].resize(lim);
                        for(int j=0;j<lim;++j){
                            if(robot_map_connectivity[i][j] == 0){
                                robot_map_distance[i][j] = 1e100;
                            }
                            else{
                                robot_map_distance[i][j] =
                            sqrt((robot_centre_point[i].first - robot_centre_point[j].first)*(robot_centre_point[i].first - robot_centre_point[j].first) + 
                                 (robot_centre_point[i].second - robot_centre_point[j].second)*(robot_centre_point[i].second - robot_centre_point[j].second));
                            }
                        }
                    }

                    for(int k=0;k<lim;++k)
                        for(int i=0;i<lim;++i)
                            for(int j=0;j<lim;++j)
                            robot_map_distance[i][j] = std::min(robot_map_distance[i][j],robot_map_distance[i][k]+robot_map_distance[k][j]);

                    base_angle = 0;

                    // RCLCPP_INFO(rclcpp::get_logger("chase_distance_limit")," ? ? %lf %lf",chase_distance_able,chase_distance_limit);
                    // chase_distance_limit = 18
                    // chase_distance_able = 3;
                }

    void DecideEnemy::message_callback_base_angle(const std_msgs::msg::Float64 &msg){
        base_angle = msg.data;
    }

    void DecideEnemy::message_callback_self_id(const std_msgs::msg::Int32 &msg){
        self_id = msg.data;
    }

    void DecideEnemy::message_callback_pos(const robot_msgs::msg::AutoaimInfo &msg){
        auto robot_pos_array = msg.data;
        std::vector<int> robot_id_array;
        for(auto element:msg.data){
            robot_timer_map[element.id] = rclcpp::Clock().now();
            robot_pos_map[element.id] = element.pos;

            if(self_id/100 == element.id/100)
            continue;
            if(robot_blood_map.find(element.id) != robot_blood_map.end()){
                if(robot_blood_map[element.id] <= 0){
                    robot_id_array.push_back(element.id);
                    continue;
                }
            }
        }
        for(auto element:robot_pos_map){
            if((rclcpp::Clock().now() - robot_timer_map[element.first]).seconds()  > 2)
            robot_id_array.push_back(element.first);
        }
        for(auto element:robot_id_array)
            robot_pos_map.erase(element);
        return;
    }

    void DecideEnemy::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_msgs::msg::AutoaimInfo new_msg;
        new_msg.data.clear();
        for(auto element:msg.data){
            geometry_msgs::msg::Point point,geopoint;
            robot_msgs::msg::RobotInfo new_element;
            point = element.pos;

            geopoint.x = point.x*cos(base_angle - My_yaw) - point.y*sin(base_angle - My_yaw) + My_pos.x;
            geopoint.y = point.x*sin(base_angle - My_yaw) + point.y*cos(base_angle - My_yaw) + My_pos.y;
            geopoint.z = point.z + My_pos.z;

            new_element.pos = geopoint;
            new_element.id = element.id;

            new_msg.data.push_back(new_element);
        }
        return message_callback_pos(new_msg);
    }


    void DecideEnemy::message_callback_radar_independent(const robot_msgs::msg::AutoaimInfo &msg){
        return message_callback_pos(msg);
    }

    void DecideEnemy::message_callback_enemy_blood(const robot_msgs::msg::RobotBloodInfo &msg){
        robot_blood_array = msg.data;
        for(auto element:robot_blood_array){
            robot_blood_map[element.id] = element.blood;
        }
        return;
    }

    void DecideEnemy::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        //Header header
        //string child_frame_id
        // geometry_msgs/TwistWithCovariance twist

        tf2::Quaternion imu_quat(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(imu_quat);
        m.getRPY(roll, pitch, yaw);

        My_yaw = yaw;
        My_pos = msg.pose.pose.position;
        My_pos.x += self_start_point.first;
        My_pos.y += self_start_point.second;
        return;
    }
    
    double Distance(double x1,double y1,double x2,double y2){
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1+y2));
    }
    
    double Critical(int blood,double distance,double chase_or_not){
        return -(blood+60*distance+chase_or_not*1000);//(改)
    }

    BT::NodeStatus DecideEnemy::tick(){
        std::string my_str;
        getInput<std::string>("priority_input",my_str);
        getInput("chase_distance_limit",chase_distance_limit);
        getInput("chase_distance_able",chase_distance_able);
        
        const auto toml_priority_file = toml::parse(ROOT "config/node_information.toml");
        robot_priority = toml::find<std::vector< std::vector< int > > >(toml_priority_file,my_str.c_str());

        std::vector<int> vec;

        rclcpp::spin_some(node);
        robot_msgs::msg::AutoaimInfo msg;
        message_callback_pos(msg);
        int Id = -1,Blood,enemy_Id;

        if(robot_blood_array.empty()) return BT::NodeStatus::FAILURE;
        if(robot_pos_map.empty()) return BT::NodeStatus::FAILURE;


        // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%lf %lf",My_pos.x,My_pos.y);

        for(int i=0;i<robot_map_field.size();++i){
            self_point = std::pair<double,double>(My_pos.x,My_pos.y);
            // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d",i);
            if(decision_utils::judging_point::judging_point(self_point,robot_map_field[i])){
                Id = i;
                break;
            }
        }
        
        // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d",Id);
        
        if(Id==-1)
        return BT::NodeStatus::FAILURE;
        

        std::pair<double,double> enemy_point;

        double rate_val = 1e100,now_val = 0;
        bool remember_chase_or_not = 0;
        int target_enemy_id = -1;

        std::vector<double> enemy_priority_vector;
        enemy_priority_vector.resize(8);

        for(int i = 0; i < 8; ++i)    enemy_priority_vector[i] = -100;

        int init_val = 1000000;
        int step_val = 10000;

        std::vector<int> enemy_priority_ranking,enemy_priority_mapping;

        for(auto priority_vector:robot_priority){
            init_val -= step_val;
            for(auto id:priority_vector){
                for(auto element:robot_pos_map)if(element.first % 9 == id){
                    enemy_point = std::pair<double,double>(element.second.x,element.second.y);
                    enemy_Id = -1;
                    // RCLCPP_INFO(rclcpp::get_logger("logger"),"%lf %lf",element.second.x,element.second.y);
                    for(int i=0;i<robot_map_field.size();++i){
                        if(decision_utils::judging_point::judging_point(enemy_point,robot_map_field[i])){
                            enemy_Id = i;
                            break;
                        }
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%lf %lf",enemy_point.first,enemy_point.second);

                    if(enemy_Id == -1) continue;

                    double  Euclid_distance = sqrt((self_point.first-enemy_point.first)*(self_point.first-enemy_point.first)+(self_point.second-enemy_point.second)*(self_point.second-enemy_point.second)),
                            true_distanece = robot_map_distance[Id][enemy_Id];

                    
                    bool    Euclid_Able = (Euclid_distance < chase_distance_able),
                            True_Able = (true_distanece < chase_distance_limit);
                    bool chase_or_not = (!((robot_graph_height[Id] + 0.1 > robot_graph_height[enemy_Id]) && Euclid_Able));

                    if(decision_utils::judging_point::judging_point(enemy_point,robot_map_field[enemy_Id])){
                        now_val = -100;
                        // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d %lf %lf %lf %lf",enemy_Id,Euclid_distance,chase_distance_able,true_distanece,chase_distance_limit);
                        if(Euclid_Able || True_Able){
                            now_val = Critical(robot_blood_map[element.first],robot_map_distance[Id][enemy_Id],chase_or_not);
                            if(now_val < rate_val) target_enemy_id = element.first,rate_val = now_val,remember_chase_or_not = chase_or_not;
                        }
                    // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d %d %lf %lf",Id,enemy_Id,robot_map_distance[Id][enemy_Id],now_val);
                        enemy_priority_vector[decision_utils::id_mapping::autoaim2priority(element.first)] = now_val + init_val;
                    }
                }
            }
        }

        // for(int i=0;i<8;++i){
        //     RCLCPP_INFO(rclcpp::get_logger("my_logger"),"enemy_id,prio:%d %lf",i,enemy_priority_vector[i]);
        // }
        for(int i=1;i<=8;++i) enemy_priority_ranking.push_back(i);
        sort(enemy_priority_ranking.begin(),enemy_priority_ranking.end(),[&](int a,int b){return enemy_priority_vector[a] > enemy_priority_vector[b];});
        enemy_priority_mapping.resize(8);
        int priority_level = 8;
        for(int i=0;i<8;++i){
            enemy_priority_mapping[enemy_priority_ranking[i]-1] = priority_level - i;
        }

        // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d ?",target_enemy_id);

        // for(int i=0;i<8;++i){
        //     RCLCPP_INFO(rclcpp::get_logger("my_logger"),"%d",enemy_priority_mapping[i]);
        // }
        // RCLCPP_INFO(rclcpp::get_logger("my_logger"),"");


        /*for(auto element:robot_pos_map){
            enemy_point = std::pair<double,double>(element.second.x,element.second.y);
            enemy_Id = -1;
            // RCLCPP_INFO(rclcpp::get_logger("logger"),"%lf %lf",element.second.x,element.second.y);
            for(int i=0;i<robot_map_field.size();++i){
                if(decision_utils::judging_point::judging_point(enemy_point,robot_map_field[i])){
                    enemy_Id = i;
                    break;
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("my_logger"),"enemy_Id:%d",enemy_Id);

            if(enemy_Id == -1) continue;
            bool FLAG = (sqrt((self_point.first-enemy_point.first)*(self_point.first-enemy_point.first)+
            (self_point.second-enemy_point.second)*(self_point.second-enemy_point.second)) < chase_distance_able);
            bool chase_or_not = (!((robot_graph_height[Id] + 0.1 > robot_graph_height[enemy_Id]) && FLAG));

            if(decision_utils::judging_point::judging_point(enemy_point,robot_map_field[enemy_Id])){
                int flag = 0;

                for(auto priority_vector:robot_priority){
                    for(auto id:priority_vector){
                        if(element.first % 100 == id){
                            if(robot_map_distance[Id][enemy_Id] < chase_distance_limit || FLAG){
                                now_val = Critical(robot_blood_map[element.first],robot_map_distance[Id][enemy_Id],chase_or_not);
                                flag = 1;
                                if(now_val < rate_val) target_enemy_id = element.first,rate_val = now_val,remember_chase_or_not = chase_or_not;
                            }
                        }
                    }
                    if(flag)
                    {
                        setOutput<int>("target_enemy_id",element.first);
                        setOutput<int>("chase_or_not",chase_or_not);
                        return BT::NodeStatus::SUCCESS;
                    }
                }
                return BT::NodeStatus::FAILURE;
            }
        }*/
        
        
        setOutput<int>("target_enemy_id",target_enemy_id);
        setOutput<int>("chase_or_not",remember_chase_or_not);
        setOutput<std::vector<int>>("priority_output",enemy_priority_mapping);

        if(target_enemy_id == -1)
        return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::DecideEnemy>("DecideEnemy");
// }
