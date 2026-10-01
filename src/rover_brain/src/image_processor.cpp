#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <opencv2/opencv.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



class ImageProcessor : public rclcpp::Node {
    public:
        ImageProcessor() : Node("image_processor"), camera_model_initialized_(false)
        {
            
            // Subscribe to the camera info topic
            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "camera/camera_info", 10,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) -> void {
                    if (!camera_model_initialized_) {
                        camera_model_.fromCameraInfo(msg);
                        camera_model_initialized_ = true;
                        RCLCPP_INFO(this->get_logger(), "Camera model initialized.");
                    }
                }
            );

            // Subscribe to the image topic
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/image_raw", 10,
                [this](const sensor_msgs::msg::Image::SharedPtr image){this->image_subscription_callback_(image);}
            );

            artifact_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_artifact_poses", 10);
            rover_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_rover_poses", 10);


            // Initialize the TF2 buffer and listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            RCLCPP_INFO(this->get_logger(), "Image Processor Node initialized.");
        }
    
    private:
        bool camera_model_initialized_;
        float artifact_real_world_height_ = 0.2; // real-world height of the artifact in meters (adjust as needed)
        float rover_real_world_height_ = 0.3; // real-world height of the rover in meters (adjust as needed)
        image_geometry::PinholeCameraModel camera_model_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr artifact_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rover_pose_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        void image_subscription_callback_(const sensor_msgs::msg::Image::SharedPtr image_msg) {
            if (!camera_model_initialized_) {
                RCLCPP_WARN(this->get_logger(), "Camera model not initialized yet. Skipping image processing.");
                return;
            }

            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // Convert the image to HSV color space for color-based detection
            // this seperates the color information from the intensity information,
            // making it easier to detect specific colors under varying lighting conditions
            cv::Mat hsv_img;
            cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

            // --- COLOR THRESHOLDS (HSV) ---
            // Blue artifact 
            cv::Scalar blue_low(100, 150, 50);
            cv::Scalar blue_high(140, 255, 255);
            // yellow rover Cuboids
            cv::Scalar yellow_low(15, 100, 100);
            cv::Scalar yellow_high(35, 255, 255);

            detect_object(hsv_img, image_msg->header, blue_low, blue_high, artifact_real_world_height_, 200.0, artifact_pose_pub_); // Detect artifacts
            detect_object(hsv_img, image_msg->header, yellow_low, yellow_high, rover_real_world_height_, 400.0, rover_pose_pub_); // Detect rovers
            
        }

        void detect_object(const cv::Mat& image, const std_msgs::msg::Header& image_header, const cv::Scalar& lower_bound, const cv::Scalar& upper_bound,
            const float real_world_height, const float min_contour_area, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher) {
            
            cv::Mat mask;
            cv::inRange(image, lower_bound, upper_bound, mask);

            // morphological opening to remove noise / terrain shadows
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            float max_area = 0.0;
            cv::Rect best_bbox;
            cv::Point2d best_centroid;
            bool found = false;

            for (const auto& contour : contours) {
                float area = cv::contourArea(contour);
                if (area > min_contour_area && area > max_area) {
                    max_area = area;
                    cv::Moments m = cv::moments(contour);
                    if (m.m00 != 0) {
                        best_centroid = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
                        best_bbox = cv::boundingRect(contour);
                        found = true;
                    }
                }
            }

            if (found) {
                // ray projection in camera optical frame
                cv::Point2d rectified_pt = camera_model_.rectifyPoint(best_centroid);
                cv::Point3d ray = camera_model_.projectPixelTo3dRay(rectified_pt);

                // distance estimation using pinhole camera geometry: Z = (Real_Height * fy) / bbox_height
                float focal_length_y = camera_model_.fy();
                float bbox_height_px = best_bbox.height;
                float estimated_z = (real_world_height * focal_length_y) / bbox_height_px;

                // PoseStamped in optical frame
                geometry_msgs::msg::PoseStamped optical_pose;
                optical_pose.header.stamp = image_header.stamp;
                optical_pose.header.frame_id = image_header.frame_id; // e.g. "rover_01/camera_link_optical"

                optical_pose.pose.position.x = ray.x * estimated_z;
                optical_pose.pose.position.y = ray.y * estimated_z;
                optical_pose.pose.position.z = ray.z * estimated_z;
                optical_pose.pose.orientation.w = 1.0;

                // transform pose to rover base_link frame
                geometry_msgs::msg::PoseStamped base_pose;
                if (transform_to_rover_base_coordinate_frame(optical_pose, base_pose)) {
                    publisher->publish(base_pose);
                }
            }
        }

        bool transform_to_rover_base_coordinate_frame(const geometry_msgs::msg::PoseStamped& input_pose, geometry_msgs::msg::PoseStamped& output_pose){
            std::string optical_frame = input_pose.header.frame_id;
            std::string target_frame;

            size_t pos = optical_frame.find("camera_link_optical");
            if (pos != std::string::npos) {
                target_frame = optical_frame;
                target_frame.replace(pos, std::string("camera_link_optical").length(), "base_link");
            } else {
                target_frame = "base_link";
            }

            try {
                output_pose = tf_buffer_->transform(input_pose, target_frame, tf2::durationFromSec(0.2));
                return true;
            } 
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF transform failed from %s to %s: %s",
                    optical_frame.c_str(), target_frame.c_str(), ex.what());
                return false;
            }
        }
};


int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageProcessor>());
	rclcpp::shutdown();
	return 0;
}
