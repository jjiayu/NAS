#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include "constants.hpp"
#include "tree.hpp"
#include "utils.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

using namespace nas;

int main() {
    // Simple YAML test
    try {
        // Get the project root directory (two levels up from the executable)
        std::filesystem::path exe_path = std::filesystem::current_path();
        std::filesystem::path config_path = exe_path.parent_path().parent_path() / "NAS" / "config" / "test.yaml";
        
        std::cout << "Looking for config file at: " << config_path << std::endl;
        
        YAML::Node config = YAML::LoadFile(config_path.string());
        
        std::cout << "Configuration Name: " << config["name"].as<std::string>() << std::endl;
        std::cout << "Version: " << config["version"].as<double>() << std::endl;
        std::cout << "Value1: " << config["parameters"]["value1"].as<int>() << std::endl;
        std::cout << "Value2: " << config["parameters"]["value2"].as<double>() << std::endl;
        std::cout << "Message: " << config["parameters"]["message"].as<std::string>() << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
        return 1;
    }

    //Create the tree
    Tree tree;

    // Expand the tree (Depth = num_steps)
    tree.expand(tree.num_steps);
    std::cout << std::endl;
    
    // Print information about each layer
    for (size_t i = 0; i < tree.layers.size(); ++i) {
        std::cout << "Layer " << i << " has " << tree.layers[i].size() 
                  << " nodes" << std::endl;
        
        // Print node IDs in this layer
        std::cout << "Node IDs: ";
        for (const auto& node : tree.layers[i]) {
            std::cout << node->node_id << " ";
        }
        std::cout << std::endl;
    }

    // Check if goal is inside which patches

    return 0;
}