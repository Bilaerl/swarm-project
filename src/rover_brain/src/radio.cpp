#include <cmath>
#include <string>
#include <chrono>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <custom_interfaces/msg/polar_vector.hpp>



class Radio : public rclcpp::Node
{
    public:
        Radio(const std::string& rover_name) : Node("minimal_publisher"), rover_name_(rover_name)
        {

            // Initialize the TF2 buffer and listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            nest_vector_publisher_ = this->create_publisher<custom_interfaces::msg::PolarVector>("radio/nest_vector", 10);
            nest_vector_publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){this->nest_vector_publisher_callback_();});

            RCLCPP_INFO(this->get_logger(), "Radio Node initialized.");
        }

    private:
        const std::string rover_name_;
        rclcpp::TimerBase::SharedPtr nest_vector_publisher_timer_;
        rclcpp::Publisher<custom_interfaces::msg::PolarVector>::SharedPtr nest_vector_publisher_;
        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        void nest_vector_publisher_callback_() {
            float nest_r, nest_theta;
            
            try {
                // look up transform from the rover's base_link to the nest_link
                std::string target_frame = rover_name_ + "/base_link";
                geometry_msgs::msg::TransformStamped trans;
                trans = tf_buffer_->lookupTransform(target_frame, "nest_link", tf2::TimePoint(std::chrono::seconds(0)));

                float dx = trans.transform.translation.x;
                float dy = trans.transform.translation.y;
                
                nest_r = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
                nest_theta = std::atan2(dy, dx);
            
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s failed to locate nest: %s", rover_name_.c_str(), ex.what());
                nest_r = std::numeric_limits<float>::infinity();
                nest_theta = std::numeric_limits<float>::infinity();
            }

            custom_interfaces::msg::PolarVector nest_vector_msg;
            nest_vector_msg.range = nest_r;
            nest_vector_msg.theta = nest_theta;
            
            nest_vector_publisher_->publish(nest_vector_msg);
    
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: radio <rover_name>");
        return 1;
    }

    std::string rover_name = argv[1];

    rclcpp::spin(std::make_shared<Radio>(rover_name));
    rclcpp::shutdown();
    return 0;
}