#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/pick_artifact.hpp"
#include "custom_interfaces/srv/drop_artifact.hpp"
#include "custom_interfaces/srv/remove_artifact.hpp"
#include "custom_interfaces/srv/spawn_artifact.hpp"

using namespace std::chrono_literals;


class Picker : public rclcpp::Node
{
	public:
		Picker()
		: Node("picker")
		{
			// the rover_name parameter can be removed when the rover is physical
			// it is only needed for simulation to pass the rover's name to the artifact manager
			this->declare_parameter<std::string>("rover_name", "");

			// services
			pick_artifact_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			pick_artifact_service_ = this->create_service<custom_interfaces::srv::PickArtifact>("picker/pick_artifact", [this](const std::shared_ptr<custom_interfaces::srv::PickArtifact::Request> request,
				std::shared_ptr<custom_interfaces::srv::PickArtifact::Response> response) {this->pick_artifact_callback(request, response);},
				rclcpp::QoS(rclcpp::ServicesQoS()),
				pick_artifact_callback_group_
			);

			drop_artifact_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			drop_artifact_service_ = this->create_service<custom_interfaces::srv::DropArtifact>("picker/drop_artifact", [this](const std::shared_ptr<custom_interfaces::srv::DropArtifact::Request> request,
				std::shared_ptr<custom_interfaces::srv::DropArtifact::Response> response) {this->drop_artifact_callback(request, response);},
				rclcpp::QoS(rclcpp::ServicesQoS()),
				drop_artifact_callback_group_
			);
			
			// client
			remove_artifact_client_ = this->create_client<custom_interfaces::srv::RemoveArtifact>("/artifact_manager/remove_artifact");
			spawn_artifact_client_ = this->create_client<custom_interfaces::srv::SpawnArtifact>("/artifact_manager/spawn_artifact");
		}


	private:
		rclcpp::CallbackGroup::SharedPtr pick_artifact_callback_group_;
		rclcpp::Service<custom_interfaces::srv::PickArtifact>::SharedPtr pick_artifact_service_;

		rclcpp::CallbackGroup::SharedPtr drop_artifact_callback_group_;
		rclcpp::Service<custom_interfaces::srv::DropArtifact>::SharedPtr drop_artifact_service_;
		
		rclcpp::Client<custom_interfaces::srv::RemoveArtifact>::SharedPtr remove_artifact_client_;
		rclcpp::Client<custom_interfaces::srv::SpawnArtifact>::SharedPtr spawn_artifact_client_;

		void pick_artifact_callback(const std::shared_ptr<custom_interfaces::srv::PickArtifact::Request> request,
				std::shared_ptr<custom_interfaces::srv::PickArtifact::Response> response) {
			
			// implement the logic for picking an artifact here
			// the current implementation is targeted towards simulation, 
			// where the artifact is removed from Gazebo and the artifact manager's list of active artifacts
			// in a physical rover, the logic would involve controlling the rover's arm to pick up the artifact
			RCLCPP_INFO(this->get_logger(), "Received request to pick artifact at (%.2f, %.2f, %.2f)",
				request->artifact_x, request->artifact_y, request->artifact_z);
			
			auto remove_request = std::make_shared<custom_interfaces::srv::RemoveArtifact::Request>();

			std::string rover_name = this->get_parameter("rover_name").as_string();

			remove_request->rover_name = rover_name;
			remove_request->artifact_x = request->artifact_x;
			remove_request->artifact_y = request->artifact_y;
			remove_request->artifact_z = request->artifact_z;

			auto remove_future = remove_artifact_client_->async_send_request(remove_request);

			auto wait_result = remove_future.wait_for(std::chrono::seconds(2));

			if (wait_result != std::future_status::ready) {
				RCLCPP_ERROR(this->get_logger(), "Timed out waiting for artifact manager response.");
				response->success = false;
				return;
			}

			auto remove_response = remove_future.get();

			if (remove_response->success) {
				RCLCPP_INFO(this->get_logger(), "Successfully picked artifact at (%.2f, %.2f, %.2f)", 
					request->artifact_x, request->artifact_y, request->artifact_z);
				response->success = true;
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to pick artifact at (%.2f, %.2f, %.2f)", 
					request->artifact_x, request->artifact_y, request->artifact_z);
				response->success = false;
			}

		};

		void drop_artifact_callback(const std::shared_ptr<custom_interfaces::srv::DropArtifact::Request> request,
				std::shared_ptr<custom_interfaces::srv::DropArtifact::Response> response) {
			
			// implement the logic for dropping an artifact here
			// the current implementation is targeted towards simulation, 
			// where the artifact is spawned in Gazebo and added to the artifact manager's list of dropped artifacts
			// in a physical rover, the logic would involve controlling the rover's arm to drop the artifact
			RCLCPP_INFO(this->get_logger(), "Received request to drop artifact");
			
			auto spawn_request = std::make_shared<custom_interfaces::srv::SpawnArtifact::Request>();

			std::string rover_name = this->get_parameter("rover_name").as_string();

			spawn_request->rover_name = rover_name;
			spawn_request->spawn_x = 1.0; // spawn the artifact 1 meter in front of the rover's base_link frame
			spawn_request->spawn_y = 0.0;
			spawn_request->spawn_z = 0.0;

			auto spawn_future = spawn_artifact_client_->async_send_request(spawn_request);

			auto wait_result = spawn_future.wait_for(std::chrono::seconds(2));

			if (wait_result != std::future_status::ready) {
				RCLCPP_ERROR(this->get_logger(), "Timed out waiting for artifact manager response.");
				response->success = false;
				return;
			}

			auto spawn_response = spawn_future.get();

			if (spawn_response->success) {
				RCLCPP_INFO(this->get_logger(), "Artifact successfully dropped");
				response->success = true;
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to drop artifact");
				response->success = false;
			}

		};

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<Picker>();

	// a MultiThreadedExecutor is required here: pick_artifact_callback blocks its thread waiting
	// on remove_artifact_client_'s future, and that future can only be fulfilled once the executor
	// processes the incoming response on a different thread. With a single-threaded executor, this would deadlock the node.
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	
	rclcpp::shutdown();
	return 0;
}