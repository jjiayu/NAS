#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// Simple Node structure for grid-based A*
struct Node {
    int x, y;           // Grid coordinates
    double g_cost;      // Cost from start to current
    double h_cost;      // Heuristic cost to goal
    double f_cost;      // Total cost (g + h)
    Node* parent;       // Parent node in the path

    Node(int x, int y, double g, double h, Node* p = nullptr) 
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    // For priority queue comparison
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

// Calculate heuristic (Euclidean distance)
double calculateHeuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Get valid neighbors for a node
std::vector<std::pair<int, int>> getNeighbors(int x, int y, const std::vector<std::vector<int>>& grid) {
    std::vector<std::pair<int, int>> neighbors;
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Check all 8 directions
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int i = 0; i < 8; i++) {
        int new_x = x + dx[i];
        int new_y = y + dy[i];
        
        // Check if the neighbor is valid (within bounds and not an obstacle)
        if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols && grid[new_x][new_y] == 0) {
            neighbors.push_back({new_x, new_y});
        }
    }
    
    return neighbors;
}

// A* search algorithm
std::vector<std::pair<int, int>> aStarSearch(const std::vector<std::vector<int>>& grid,
                                            int start_x, int start_y,
                                            int goal_x, int goal_y) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Priority queue for open set
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open_set;
    
    // Hash map for closed set
    std::unordered_map<int, Node*> closed_set;
    
    // Create start node
    Node* start_node = new Node(start_x, start_y, 0, 
                              calculateHeuristic(start_x, start_y, goal_x, goal_y));
    open_set.push(start_node);
    
    while (!open_set.empty()) {
        // Get node with lowest f_cost
        Node* current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current->x == goal_x && current->y == goal_y) {
            // Reconstruct path
            std::vector<std::pair<int, int>> path;
            Node* node = current;
            while (node != nullptr) {
                path.push_back({node->x, node->y});
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            
            // Clean up memory
            for (auto& pair : closed_set) {
                delete pair.second;
            }
            while (!open_set.empty()) {
                delete open_set.top();
                open_set.pop();
            }
            
            return path;
        }
        
        // Add to closed set (using a unique key for the coordinates)
        closed_set[current->x * cols + current->y] = current;
        
        // Check neighbors
        for (const auto& neighbor : getNeighbors(current->x, current->y, grid)) {
            int nx = neighbor.first;
            int ny = neighbor.second;
            
            // Skip if in closed set
            if (closed_set.find(nx * cols + ny) != closed_set.end()) {
                continue;
            }
            
            // Calculate new g_cost (diagonal movement costs more)
            double new_g_cost = current->g_cost + 
                (nx != current->x && ny != current->y ? 1.414 : 1.0);
            
            // Create new node
            Node* neighbor_node = new Node(
                nx, ny,
                new_g_cost,
                calculateHeuristic(nx, ny, goal_x, goal_y),
                current
            );
            
            // Add to open set
            open_set.push(neighbor_node);
        }
    }
    
    // No path found
    return std::vector<std::pair<int, int>>();
}

int main() {
    
    // Create a simple grid (0 = free space, 1 = obstacle)
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };
    
    // Define start and goal positions
    int start_x = 0, start_y = 0;
    int goal_x = 4, goal_y = 4;
    
    // Run A* search
    std::vector<std::pair<int, int>> path = aStarSearch(grid, start_x, start_y, goal_x, goal_y);
    
    // Print the path
    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        for (const auto& point : path) {
            std::cout << "(" << point.first << ", " << point.second << ") ";
        }
        std::cout << std::endl;
    }
    
    return 0;
} 