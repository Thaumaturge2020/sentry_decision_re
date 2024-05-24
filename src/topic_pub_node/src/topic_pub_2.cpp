#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/referee_data.hpp"
#include "robot_msgs/msg/gimbal_data.hpp"
#include "robot_msgs/msg/int_array.hpp"
#include "toml.hpp"


using namespace std::chrono_literals;

namespace topic_pub_node_2 {
    class TopicPublisher_2 : public rclcpp::Node {
    public :
        rclcpp::Publisher<robot_msgs::msg::GimbalData>::SharedPtr publisher_1;
        rclcpp::Publisher<robot_msgs::msg::RefereeData>::SharedPtr publisher_2;
        rclcpp::Subscription<robot_msgs::msg::IntArray>::SharedPtr subscription_info;

        std::vector<int> blood_array;
        int game_time,allow_bullet,robot_id,vitual_mode,wait_seconds;
        bool use_time;
        rclcpp::Time time_start;

        void reinit(){
            const auto toml_file = toml::parse(ROOT "config/config.toml");
            game_time = toml::find<int>(toml_file,"game_time");
            allow_bullet = toml::find<int>(toml_file,"allow_bullet");
            robot_id = toml::find<int>(toml_file,"robot_id");
            blood_array = toml::find<std::vector<int>>(toml_file,"blood_array");
            vitual_mode = toml::find<int>(toml_file,"vitual_mode");
            use_time = toml::find<int>(toml_file,"use_time");
            wait_seconds = toml::find<int>(toml_file,"wait_seconds");
            return;
        }

        
        TopicPublisher_2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                :   Node("TopicPublisher_2"),
                    timer_(this->create_wall_timer(std::chrono::milliseconds(1000), [this] { behaviour_publish(); })) {
                    publisher_1 = this->create_publisher<robot_msgs::msg::GimbalData>("/easy_robot_commands/offset_data",10);
                    publisher_2 = this->create_publisher<robot_msgs::msg::RefereeData>("/easy_robot_commands/referee_data_for_decision",10);
                    reinit();
                    subscription_info = this->create_subscription<robot_msgs::msg::IntArray>("/easy_robot_commands/topic_pub_2",10,std::bind(&TopicPublisher_2::decision_callback,this,std::placeholders::_1));
                    RCLCPP_INFO(rclcpp::get_logger("topic_publisher_logger"),"TopicPublisher_2 has been started. %d %d %d",game_time,allow_bullet,robot_id);
                    time_start = rclcpp::Clock().now();
        }

        void decision_callback(const robot_msgs::msg::IntArray::SharedPtr msg){
            RCLCPP_INFO(rclcpp::get_logger("topic_publisher_logger"),"Successfully received decision info");
            if(msg->data.size()!=2) return;
            int id = msg->data[0],val = msg->data[1];
            if(id < 0){
                id = -id;
                switch (id){
                    case 1:
                        game_time = val;
                        break;
                    case 2:
                        allow_bullet = val;
                        break;
                    case 3:
                        robot_id = val;
                        break;
                    case 4:
                        vitual_mode = val;
                        break;
                    default:
                        break;
                }
            }
            else{
                if(id >= 0 && id < 16){
                    blood_array[id] = val;
                }
            }
        }

        void behaviour_publish() {
            reinit();
            std_msgs::msg::Int32 msg;
            robot_msgs::msg::AutoaimInfo msg2;
            robot_msgs::msg::RobotBloodInfo msg3;
            robot_msgs::msg::RobotInfo data1;
            robot_msgs::msg::RobotBattleState data2;

            robot_msgs::msg::GimbalData gimbal_msg;
            robot_msgs::msg::RefereeData referee_msg;

            data1.id = 0,data1.pos.x = 8,data1.pos.y = 8,data1.pos.z = 0;
            
            if((rclcpp::Clock().now() - time_start).seconds() < wait_seconds)
            referee_msg.game_time = -1;
            else if(use_time == 1)
            referee_msg.game_time = game_time;
            else
            referee_msg.game_time = std::max(0,(int)(rclcpp::Clock().now() - time_start).seconds() - wait_seconds);
            referee_msg.allow_bullet = allow_bullet;
            referee_msg.robot_id = robot_id;
            gimbal_msg.virtual_mode = vitual_mode;
            gimbal_msg.offset = 0;
            // RCLCPP_INFO(rclcpp::get_logger("logger"),"%d %d",vitual_mode,gimbal_msg.virtual_mode);
            data2.id = 1,data2.blood = blood_array[0];
            referee_msg.data.push_back(data2);
            data2.id = 2,data2.blood = blood_array[1];
            referee_msg.data.push_back(data2);
            data2.id = 3,data2.blood = blood_array[2];
            referee_msg.data.push_back(data2);
            data2.id = 4,data2.blood = blood_array[3];
            referee_msg.data.push_back(data2);
            data2.id = 5,data2.blood = blood_array[4];
            referee_msg.data.push_back(data2);
            data2.id = 7,data2.blood = blood_array[5];
            referee_msg.data.push_back(data2);
            data2.id = 8,data2.blood = blood_array[6];
            referee_msg.data.push_back(data2);
            data2.id = 9,data2.blood = blood_array[7];
            referee_msg.data.push_back(data2);

            data2.id = 101,data2.blood = blood_array[8];
            referee_msg.data.push_back(data2);
            data2.id = 102,data2.blood = blood_array[9];
            referee_msg.data.push_back(data2);
            data2.id = 103,data2.blood = blood_array[10];
            referee_msg.data.push_back(data2);
            data2.id = 104,data2.blood = blood_array[11];
            referee_msg.data.push_back(data2);
            data2.id = 105,data2.blood = blood_array[12];
            referee_msg.data.push_back(data2);
            data2.id = 107,data2.blood = blood_array[13];
            referee_msg.data.push_back(data2);
            data2.id = 108,data2.blood = blood_array[14];
            referee_msg.data.push_back(data2);
            data2.id = 109,data2.blood = blood_array[15];
            referee_msg.data.push_back(data2);
            
            gimbal_msg.offset = 0;
            publisher_1->publish(gimbal_msg);
            publisher_2->publish(referee_msg);
        }

    private :
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(topic_pub_node_2::TopicPublisher_2)
