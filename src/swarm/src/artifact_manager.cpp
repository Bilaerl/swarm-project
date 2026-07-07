#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "swarm/srv/remove_artifact.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"

using namespace std::chrono_literals;


struct Artifact
{
    std::string name;
    float x;
    float y;
    float z;
    std::optional<std::string> picked_by = std::nullopt; // name of the rover that picked the artifact, empty if not picked
};


class ArtifactManager : public rclcpp::Node
{
	public:
		ArtifactManager()
		: Node("artifact_manager"), matching_threshold_(0.2f)
		{
			// logic to look for all active artifact and store them in active_artifacts_
            this->declare_parameter<std::string>("artifacts_spawn_config_file_path", "");

            std::string file_path;
            this->get_parameter("artifacts_spawn_config_file_path", file_path);

            if (!file_path.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Successfully received static artifacts file path: %s", file_path.c_str());
                extract_artifacts_from_file(file_path);
            } else {
                RCLCPP_WARN(this->get_logger(), "Artifacts file path parameter is empty!");
            }
            
            // start servers for picking, and dropping artifacts respectively

            // gz_delete_client_ is placed on its own mutually-exclusive callback group so its
            // future can be resolved by a different executor thread than the one blocked waiting
            // on it inside remove_artifact_callback (see main(), which uses a MultiThreadedExecutor).
            client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            gz_delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(
                "/world/swarm_world/remove",
                rclcpp::QoS(rclcpp::ServicesQoS()),
                client_callback_group_);

            artifact_removal_service_ = this->create_service<swarm::srv::RemoveArtifact>(
                "artifact_manager/remove_artifact", [this](const std::shared_ptr<swarm::srv::RemoveArtifact::Request> request,
                std::shared_ptr<swarm::srv::RemoveArtifact::Response> response) {this->remove_artifact_callback(request, response);});
            
            
            RCLCPP_INFO(this->get_logger(), "Swarm artifact tracking system online.");

        }


	private:
        const float matching_threshold_;
        std::mutex artifact_mutex_;
        std::vector<Artifact> active_artifacts_;
        std::vector<Artifact> picked_artifacts_;
        std::vector<Artifact> dropped_artifacts_;

        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr gz_delete_client_;
        rclcpp::Service<swarm::srv::RemoveArtifact>::SharedPtr artifact_removal_service_;
        rclcpp::CallbackGroup::SharedPtr client_callback_group_;

        
        void extract_artifacts_from_file(const std::string& file_path)
        {
            std::ifstream file(file_path);

            if(!file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Could not open configuration file at: %s", file_path.c_str());
                return;
            }

            std::string line;
            while (std::getline(file, line))
            {
                // skip empty lines to prevent out-of-range errors
                if (line.empty()) {
                    continue;
                }

                // skip comments (lines starting with '#')
                if (line[0] == '#') {
                    continue;
                }

                std::stringstream ss(line);
                Artifact artifact;

                // extract whitespace-separated values sequentially
                // Note: Coordinates are parsed directly into doubles for simulation accuracy
                if (ss >> artifact.name >> artifact.x >> artifact.y >> artifact.z) {
                    std::scoped_lock lock(artifact_mutex_);
                    active_artifacts_.push_back(artifact);
                }
            }
            
            file.close();
            RCLCPP_INFO(this->get_logger(), "Extracted %zu artifacts from file.", active_artifacts_.size());
        }

        void remove_artifact_callback(const std::shared_ptr<swarm::srv::RemoveArtifact::Request> request,
                std::shared_ptr<swarm::srv::RemoveArtifact::Response> response) {
            
            RCLCPP_INFO(this->get_logger(), "Received request from %s to pick artifact at %f, %f, %f",
                request->rover_name.c_str(), request->artifact_x, request->artifact_y, request->artifact_z);
            
            double target_x = request->artifact_x;
            double target_y = request->artifact_y;
            double target_z = request->artifact_z;

            std::optional<Artifact> target_artifact;

            {
                std::scoped_lock lock(artifact_mutex_);
                double min_dist = std::numeric_limits<double>::max();

                // loop through tracked elements to evaluate distance
                for (const auto& artifact : active_artifacts_) {

                    double dist = std::sqrt(
                        std::pow(artifact.x - target_x, 2) +
                        std::pow(artifact.y - target_y, 2) +
                        std::pow(artifact.z - target_z, 2)
                    );
                    
                    if (dist < min_dist && dist <= matching_threshold_) {
                        min_dist = dist;
                        target_artifact = artifact;
                    }
                }
                
            }

            if (target_artifact.has_value()) {
                RCLCPP_INFO(this->get_logger(), "Target artifact found, removing...");

                if (!gz_delete_client_->wait_for_service(std::chrono::seconds(1))) {
                    RCLCPP_ERROR(this->get_logger(), "Gazebo deletion bridge service unavailable.");
                    response->success = false;
                    return;
                }

                // Construct standard Gazebo removal request
                auto gz_req = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
                gz_req->entity.name = target_artifact->name;
                gz_req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;

                // Send the request and block *this* callback until it resolves, so that
                // `response` is fully populated before remove_artifact_callback returns
                // (rclcpp sends whatever is in `response` back to the caller the moment this
                // function returns — populating it later, inside an async callback, is too late).
                //
                // This blocks the thread handling artifact_removal_service_, but since
                // gz_delete_client_ lives on its own callback group (client_callback_group_),
                // a MultiThreadedExecutor can still service the client's response on a
                // different thread while this one waits. See main().
                auto gz_future = gz_delete_client_->async_send_request(gz_req);

                auto wait_result = gz_future.wait_for(std::chrono::seconds(2));

                if (wait_result != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Timed out waiting for Gazebo deletion response.");
                    response->success = false;
                    return;
                }

                auto gz_response = gz_future.get();

                if (gz_response->success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully removed artifact from Gazebo.");

                    response->success = true;

                    std::scoped_lock lock(artifact_mutex_);
                    // Remove the artifact from active_artifacts_ and add to picked_artifacts_
                    active_artifacts_.erase(std::remove_if(active_artifacts_.begin(), active_artifacts_.end(), [target_artifact](const Artifact& a) {
                        return a.name == target_artifact->name;
                    }), active_artifacts_.end());

                    Artifact picked_artifact = target_artifact.value();
                    picked_artifact.picked_by = request->rover_name;

                    picked_artifacts_.push_back(picked_artifact);

                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to remove artifact from Gazebo");
                    response->success = false;
                }

            } else {
                RCLCPP_WARN(this->get_logger(), "Target artifact not found.");
                response->success = false;
            }
            
        }
		
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ArtifactManager>();

	// A MultiThreadedExecutor is required here: remove_artifact_callback blocks its thread
	// waiting on gz_delete_client_'s future, and that future can only be fulfilled once
	// the executor processes the incoming Gazebo response on a *different* thread. With a
	// single-threaded executor (or spin()), this would deadlock the node.
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}