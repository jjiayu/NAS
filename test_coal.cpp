#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/convex.h>
#include <coal/distance.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

// Point-to-exact-convex-patch distance using COAL's convex hull
double compute_point_to_convex_patch_distance(const coal::Vec3s& point, 
                                              const std::vector<coal::Vec3s>& vertices) {
    if (vertices.size() < 3) {
        std::cerr << "Error: Need at least 3 vertices for a convex patch" << std::endl;
        return -1.0;
    }
    
    // Create a point as a very small sphere
    auto point_shape = std::make_shared<coal::Sphere>(1e-6);
    coal::CollisionObject point_obj(point_shape);
    
    // Set point position
    coal::Transform3s point_tf = coal::Transform3s::Identity();
    point_tf.translation() = point;
    point_obj.setTransform(point_tf);
    
    // Create exact convex shape using COAL's Convex class
    // Method 1: Try using the direct constructor
    try {
        // Prepare vertices and triangulation
        auto vertices_ptr = std::make_shared<std::vector<coal::Vec3s>>(vertices);
        std::vector<coal::Triangle> triangles;
        
        // Create fan triangulation from first vertex for convex polygon
        for (size_t i = 1; i < vertices.size() - 1; ++i) {
            triangles.push_back(coal::Triangle(0, i, i + 1));
        }
        auto triangles_ptr = std::make_shared<std::vector<coal::Triangle>>(triangles);
        
        // Create exact convex mesh
        auto convex_shape = std::make_shared<coal::Convex<coal::Triangle>>(
            vertices_ptr, vertices.size(),
            triangles_ptr, triangles.size()
        );
        
        coal::CollisionObject convex_obj(convex_shape);
        coal::Transform3s convex_tf = coal::Transform3s::Identity();
        convex_obj.setTransform(convex_tf);
        
        // Compute exact distance
        coal::DistanceRequest request;
        coal::DistanceResult result;
        
        coal::distance(&point_obj, &convex_obj, request, result);
        
        return result.min_distance;
        
    } catch (const std::exception& e) {
        std::cerr << "Convex shape creation failed: " << e.what() << std::endl;
        return -1.0;
    }
}

// Fallback: Point-to-triangle distance using box approximation
double compute_point_to_triangle_distance(const coal::Vec3s& point, 
                                          const coal::Vec3s& v1, const coal::Vec3s& v2, const coal::Vec3s& v3) {
    std::vector<coal::Vec3s> vertices = {v1, v2, v3};
    return compute_point_to_convex_patch_distance(point, vertices);
}

int main() {
    std::cout << "Testing COAL library..." << std::endl;
    
    // Create two simple shapes
    auto box1 = std::make_shared<coal::Box>(1.0, 1.0, 1.0);  // 1x1x1 box
    auto box2 = std::make_shared<coal::Box>(0.5, 0.5, 0.5);  // 0.5x0.5x0.5 box
    
    // Create collision objects
    coal::CollisionObject obj1(box1);
    coal::CollisionObject obj2(box2);
    
    // Set transforms (positions)
    coal::Transform3s tf1 = coal::Transform3s::Identity();
    coal::Transform3s tf2 = coal::Transform3s::Identity();
    tf2.translation() = coal::Vec3s(2.0, 0.0, 0.0);  // Move box2 to (2,0,0)
    
    obj1.setTransform(tf1);
    obj2.setTransform(tf2);
    
    // Compute distance using GJK
    coal::DistanceRequest request;
    coal::DistanceResult result;
    
    coal::distance(&obj1, &obj2, request, result);
    
    std::cout << "Distance between boxes: " << result.min_distance << std::endl;
    std::cout << "Closest point on box1: (" 
              << result.nearest_points[0][0] << ", "
              << result.nearest_points[0][1] << ", "
              << result.nearest_points[0][2] << ")" << std::endl;
    std::cout << "Closest point on box2: (" 
              << result.nearest_points[1][0] << ", "
              << result.nearest_points[1][1] << ", "
              << result.nearest_points[1][2] << ")" << std::endl;
    
    // Test point-to-convex-patch distance with various shapes
    std::cout << "\n=== Testing Point-to-Convex-Patch GJK Distance ===" << std::endl;
    
    coal::Vec3s test_point(1.0, 0.5, 0.0);  // Test point
    
    // Test 1: Triangle
    std::cout << "\n1. Triangle:" << std::endl;
    std::vector<coal::Vec3s> triangle = {
        coal::Vec3s(0.0, 0.0, 1.0),   // vertex 1
        coal::Vec3s(2.0, 0.0, 1.0),   // vertex 2
        coal::Vec3s(1.0, 2.0, 1.0)    // vertex 3
    };
    double tri_distance = compute_point_to_convex_patch_distance(test_point, triangle);
    std::cout << "Distance to triangle: " << tri_distance << std::endl;
    
    // Test 2: Square/Quadrilateral
    std::cout << "\n2. Square:" << std::endl;
    std::vector<coal::Vec3s> square = {
        coal::Vec3s(0.0, 0.0, 1.0),   // bottom-left
        coal::Vec3s(2.0, 0.0, 1.0),   // bottom-right
        coal::Vec3s(2.0, 2.0, 1.0),   // top-right
        coal::Vec3s(0.0, 2.0, 1.0)    // top-left
    };
    double square_distance = compute_point_to_convex_patch_distance(test_point, square);
    std::cout << "Distance to square: " << square_distance << std::endl;
    
    // Test 3: Pentagon
    std::cout << "\n3. Pentagon:" << std::endl;
    std::vector<coal::Vec3s> pentagon = {
        coal::Vec3s(1.0, 0.0, 1.0),     // bottom center
        coal::Vec3s(2.0, 0.5, 1.0),     // bottom right
        coal::Vec3s(1.5, 1.8, 1.0),     // top right
        coal::Vec3s(0.5, 1.8, 1.0),     // top left
        coal::Vec3s(0.0, 0.5, 1.0)      // bottom left
    };
    double pent_distance = compute_point_to_convex_patch_distance(test_point, pentagon);
    std::cout << "Distance to pentagon: " << pent_distance << std::endl;
    
    // Test 4: Hexagon
    std::cout << "\n4. Hexagon:" << std::endl;
    std::vector<coal::Vec3s> hexagon;
    double radius = 1.0;
    coal::Vec3s hex_center(1.0, 1.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        double angle = i * M_PI / 3.0;  // 60 degrees apart
        hexagon.push_back(coal::Vec3s(
            hex_center[0] + radius * cos(angle),
            hex_center[1] + radius * sin(angle),
            hex_center[2]
        ));
    }
    double hex_distance = compute_point_to_convex_patch_distance(test_point, hexagon);
    std::cout << "Distance to hexagon: " << hex_distance << std::endl;
    
    // Test 5: Octagon
    std::cout << "\n5. Octagon:" << std::endl;
    std::vector<coal::Vec3s> octagon;
    for (int i = 0; i < 8; ++i) {
        double angle = i * M_PI / 4.0;  // 45 degrees apart
        octagon.push_back(coal::Vec3s(
            hex_center[0] + radius * cos(angle),
            hex_center[1] + radius * sin(angle),
            hex_center[2]
        ));
    }
    double oct_distance = compute_point_to_convex_patch_distance(test_point, octagon);
    std::cout << "Distance to octagon: " << oct_distance << std::endl;
    
    // Test 6: Different height patch
    std::cout << "\n6. Triangle at different height:" << std::endl;
    std::vector<coal::Vec3s> high_triangle = {
        coal::Vec3s(0.5, 0.0, 2.5),   // vertex 1
        coal::Vec3s(1.5, 0.0, 2.5),   // vertex 2
        coal::Vec3s(1.0, 1.0, 2.5)    // vertex 3
    };
    double high_tri_distance = compute_point_to_convex_patch_distance(test_point, high_triangle);
    std::cout << "Distance to high triangle: " << high_tri_distance << std::endl;
    
    std::cout << "\nTest point: (" << test_point[0] << ", " 
              << test_point[1] << ", " << test_point[2] << ")" << std::endl;
    
    // Test 7: Random convex shapes
    std::cout << "\n=== Testing Random Convex Shapes ===" << std::endl;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> pos_dist(-2.0, 4.0);
    std::uniform_real_distribution<double> height_dist(0.5, 3.0);
    std::uniform_int_distribution<int> vertex_count_dist(4, 8);
    
    for (int test_num = 1; test_num <= 3; ++test_num) {
        std::cout << "\nRandom Test " << test_num << ":" << std::endl;
        
        // Generate random number of vertices
        int num_vertices = vertex_count_dist(gen);
        std::cout << "  Shape: " << num_vertices << "-sided convex polygon" << std::endl;
        
        // Generate random convex shape using circular arrangement
        std::vector<coal::Vec3s> random_vertices;
        coal::Vec3s random_center(pos_dist(gen), pos_dist(gen), height_dist(gen));
        double random_radius = std::uniform_real_distribution<double>(0.5, 1.5)(gen);
        
        for (int i = 0; i < num_vertices; ++i) {
            double angle = (2.0 * M_PI * i) / num_vertices;
            // Add small random perturbation to make it more interesting
            double radius_variation = random_radius * (0.8 + 0.4 * std::uniform_real_distribution<double>(0.0, 1.0)(gen));
            
            random_vertices.push_back(coal::Vec3s(
                random_center[0] + radius_variation * cos(angle),
                random_center[1] + radius_variation * sin(angle),
                random_center[2]
            ));
        }
        
        // Compute distance to random convex shape
        double random_distance = compute_point_to_convex_patch_distance(test_point, random_vertices);
        
        std::cout << "  Center: (" << random_center[0] << ", " << random_center[1] << ", " << random_center[2] << ")" << std::endl;
        std::cout << "  Radius: ~" << random_radius << std::endl;
        std::cout << "  Distance: " << random_distance << std::endl;
        
        // Print vertices for verification
        std::cout << "  Vertices:" << std::endl;
        for (size_t i = 0; i < random_vertices.size(); ++i) {
            const auto& v = random_vertices[i];
            std::cout << "    [" << i << "]: (" << v[0] << ", " << v[1] << ", " << v[2] << ")" << std::endl;
        }
    }
    
    std::cout << "\nCOAL test completed successfully!" << std::endl;
    
    return 0;
}
