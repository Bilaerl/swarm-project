#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"


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
		: Node("artifact_manager"), has_collected_artifacts_(false)
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
            // 4. (Optional) Initialize your ROS 2 service here to handle picking requests
            RCLCPP_INFO(this->get_logger(), "Artifact Swarm Tracking System Online.");
            

            // start servers for picking, and dropping artifacts respectively
        }


	private:
        bool has_collected_artifacts_;
        std::vector<Artifact> active_artifacts_;
        std::vector<Artifact> picked_artifacts_;
        std::vector<Artifact> dropped_artifacts_;

        
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
                    active_artifacts_.push_back(artifact);
                }
            }
            
            file.close();
            RCLCPP_INFO(this->get_logger(), "Extracted %zu artifacts from file.", active_artifacts_.size());
        }
		
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArtifactManager>());
	rclcpp::shutdown();
	return 0;
}