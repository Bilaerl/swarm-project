#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class RobotCore : public rclcpp::Node
{
    public:
        RobotCore()
        : Node("robot_core"), robot_state_(RobotState::IDLE)
        {
            // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            // auto timer_callback =
            //   [this]() -> void {
            //     auto message = std_msgs::msg::String();
            //     message.data = "Hello, world! " + std::to_string(this->count_++);
            //     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            //     this->publisher_->publish(message);
            //   };
            // timer_ = this->create_wall_timer(500ms, timer_callback);
        }

    private:
        //   rclcpp::TimerBase::SharedPtr timer_;
        //   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        
        std::vector<std::string> robot_inventory_;
        enum class RobotState { 
            IDLE,   // The robot is idle and waiting for a task
            SEARCHING,  // The robot is searching for an artifact to pick up
            PICKING,  // The robot is in the process of picking up an artifact
            RETURNING, // The robot is returning to the swarm nest with an artifact
            DROPPING  // The robot is in the process of dropping off an artifact at the swarm nest
        } robot_state_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotCore>());
  rclcpp::shutdown();
  return 0;
}