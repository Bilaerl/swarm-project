#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rover_brain/srv/pick_artifact.hpp"

using namespace std::chrono_literals;


class Picker : public rclcpp::Node
{
	public:
		Picker()
		: Node("picker")
		{
			// Create the service
			service_ = this->create_service<rover_brain::srv::PickArtifact>("picker/pick_artifact", [this](const std::shared_ptr<rover_brain::srv::PickArtifact::Request> request,
				std::shared_ptr<rover_brain::srv::PickArtifact::Response> response) {this->pick_artifact_callback(request, response);});
		}


	private:
		rclcpp::Service<rover_brain::srv::PickArtifact>::SharedPtr service_;

		void pick_artifact_callback(const std::shared_ptr<rover_brain::srv::PickArtifact::Request> request,
				std::shared_ptr<rover_brain::srv::PickArtifact::Response> response) {
			
			// Implement the logic for picking an artifact here
			RCLCPP_INFO(this->get_logger(), "Received request to pick artifact with at (%.2f, %.2f, %.2f)",
				request->artifact_x, request->artifact_y, request->artifact_z);
			
			// For demonstration, let's assume the artifact is always picked successfully
			response->success = true;
		};
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Picker>());
	rclcpp::shutdown();
	return 0;
}