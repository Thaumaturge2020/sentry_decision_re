#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_msgs/msg/aerial_commands.hpp"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/robot_blood_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "toml.hpp"


using namespace std::chrono_literals;

namespace topic_pub_node_1 {
    class TopicPublisher_1 : public rclcpp::Node {
    public :
        rclcpp::Publisher<robot_msgs::msg::AutoaimInfo>::SharedPtr publisher_1;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_2;
        rclcpp::Publisher<robot_msgs::msg::AerialCommands>::SharedPtr publisher_3;

        std::pair<double,double> self_position;
        std::vector<std::pair<double,double> > robot_position_vector;
        std::vector<int>        robot_id_vector;
        
        TopicPublisher_1(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("TopicPublisher_1"),
                  timer_(this->create_wall_timer(std::chrono::milliseconds(1000), [this] { behaviour_publish(); })) {
                  publisher_1 = this->create_publisher<robot_msgs::msg::AutoaimInfo>("/autoaim2decision", 10);
                  publisher_2 = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_Vehicle", 10);
                  publisher_3 = this->create_publisher<robot_msgs::msg::AerialCommands>("/easy_robot_commands/map_command",10);

                  const auto toml_file = toml::parse(ROOT "config/config.toml");
                  self_position = toml::find<std::pair<double,double> >(toml_file,"self_position");
                  robot_position_vector = toml::find<std::vector<std::pair<double,double> > >(toml_file,"robot_position_array");
                  robot_id_vector = toml::find<std::vector<int> >(toml_file,"robot_id_array");
        }

        void behaviour_publish() {
            std_msgs::msg::Int32 msg;
            robot_msgs::msg::AutoaimInfo msg2;
            robot_msgs::msg::RobotBloodInfo msg3;
            robot_msgs::msg::RobotInfo data1;
            robot_msgs::msg::RobotBattleState data2;
            robot_msgs::msg::AerialCommands data3;
            data2.id = 0,data2.blood = 100;
            data1.id = 1,data1.pos.x = 8,data1.pos.y = 8,data1.pos.z = 0;
            const auto toml_file = toml::parse(ROOT "config/config.toml");
            std::string input = toml::find<std::string>(toml_file,"cmd_keyboard");
            data3.cmd_keyboard = input[0];
            if(data3.cmd_keyboard == 0)
            data3.target_robot_id = 2;

            data3.target_position_x = toml::find<double>(toml_file,"target_position_x");
            data3.target_position_y = toml::find<double>(toml_file,"target_position_y");

            double start_pos_x = 2.898,start_pos_y = 3.353;
            
            msg.data = 2;
            // publisher_1->publish(msg);
            for(int i=0,lim = robot_position_vector.size();i<lim;++i){
              data1.id = robot_id_vector[i],data1.pos.x = robot_position_vector[i].first,data1.pos.y = robot_position_vector[i].second,data1.pos.z = 0;
              msg2.data.push_back(data1);
            }
            
            publisher_1->publish(msg2);
            publisher_3->publish(data3);
            // publisher_4->publish(msg);
            
            nav_msgs::msg::Odometry msg5;

            msg5.pose.pose.position.x = self_position.first - start_pos_x;
            msg5.pose.pose.position.y = self_position.second - start_pos_y;
            msg5.pose.pose.position.z = 0.0;
            publisher_2->publish(msg5);
        }

    private :
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


RCLCPP_COMPONENTS_REGISTER_NODE(topic_pub_node_1::TopicPublisher_1)
