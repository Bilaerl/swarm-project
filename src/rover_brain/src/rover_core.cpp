#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rover_brain/srv/pick_artifact.hpp"

using namespace std::chrono_literals;


class RoverCore : public rclcpp::Node
{
	public:
		RoverCore(const std::string & rover_name)
		: Node("rover_core"), rover_name_(rover_name), rover_inventory_(0), rover_state_(RoverState::IDLE)
		{
			pick_artifact_client_ = this->create_client<rover_brain::srv::PickArtifact>("picker/pick_artifact");
		}

	private:
		rclcpp::Client<rover_brain::srv::PickArtifact>::SharedPtr pick_artifact_client_;
		
		const std::string rover_name_; // Name of the rover
		const size_t max_inventory_ = 2; // Maximum number of artifacts the rover can carry at once
		size_t rover_inventory_;
		enum class RoverState { 
			IDLE,   // The rover is idle and waiting for a task
			SEARCHING,  // The rover is searching for an artifact to pick up
			PICKING,  // The rover is in the process of picking up an artifact
			RETURNING, // The rover is returning to the swarm nest with an artifact
			DROPPING  // The rover is in the process of dropping off an artifact at the swarm nest
		} rover_state_;

		void pick_artifact(int artifact_x, int artifact_y, int artifact_z) {

			if (!pick_artifact_client_->wait_for_service(1s)) {
				if (!rclcpp::ok()) {
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return;
				}
				RCLCPP_ERROR(this->get_logger(), "pick_artifact service not available. Exiting");
			}

			auto request = std::make_shared<rover_brain::srv::PickArtifact::Request>();
			request->artifact_x = artifact_x;
			request->artifact_y = artifact_y;
			request->artifact_z = artifact_z;

			auto async_request_callback = [this](rclcpp::Client<rover_brain::srv::PickArtifact>::SharedFuture future) {
				auto response = future.get();
				if (response->success) { // artifact picked sucessfully
					
					// Increment rover inventory
					rover_inventory_++;

					if (rover_inventory_ >= max_inventory_) {
						update_rover_state(RoverState::RETURNING);
						RCLCPP_INFO(this->get_logger(), "Rover inventory full. Returning to swarm nest.");
					} else {
						update_rover_state(RoverState::SEARCHING);
						RCLCPP_INFO(this->get_logger(), "Continuing to search for more artifacts.");
					}

				} else {
					// picking the artifact failed
				}
			};

			// Send the request asynchronously and use the callback to handle its response
			pick_artifact_client_->async_send_request(request, async_request_callback);

		}

		void update_rover_state(RoverState new_state) {
			rover_state_ = new_state;
		}


};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	if (argc < 2) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: rover_core <rover_name>");
		return 1;
	}

	std::string rover_name = argv[1];

	rclcpp::spin(std::make_shared<RoverCore>(rover_name));
	rclcpp::shutdown();
	return 0;
}