#include "map_loader.h"
#include "astar.h"
#include "dijkstra.h"
#include <iostream>
#include <fstream>

void printPath(const std::vector<Point>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    for (const auto& point : path) {
        file << point.x << "," << point.y << std::endl;
    }
    file.close();
    std::cout << "Path saved to: " << filename << std::endl;
}

void testAlgorithm(const std::string& algo_name,
                   const std::vector<Point>& path,
                   double path_length,
                   double compute_time,
                   bool path_found) {
    std::cout << "\n========== " << algo_name << " ==========\n";
    std::cout << "Path found: " << (path_found ? "YES" : "NO") << std::endl;

    if (path_found) {
        std::cout << "Path length: " << path_length << " grid units" << std::endl;
        std::cout << "Number of waypoints: " << path.size() << std::endl;
        std::cout << "Compute time: " << compute_time << " ms" << std::endl;
        std::cout << "Obstacle avoidance: SUCCESS" << std::endl;
    } else {
        std::cout << "Obstacle avoidance: FAILED (no valid path)" << std::endl;
        std::cout << "Compute time: " << compute_time << " ms" << std::endl;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== Path Planning Test Node ===\n" << std::endl;

    // Load map
    MapLoader map_loader;
    std::string map_file = "../map/test_map.yaml";

    std::cout << "Loading map: " << map_file << std::endl;
    if (!map_loader.loadMap(map_file)) {
        std::cerr << "Failed to load map!" << std::endl;
        return 1;
    }

    std::cout << "Map loaded successfully!" << std::endl;
    std::cout << "Map size: " << map_loader.getWidth() << " x "
              << map_loader.getHeight() << " pixels" << std::endl;
    std::cout << "Resolution: " << map_loader.getMapInfo().resolution
              << " m/pixel\n" << std::endl;

    // Define start and goal points
    Point start(200, 200);
    Point goal(300, 300);

    std::cout << "Start point: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal point: (" << goal.x << ", " << goal.y << ")\n" << std::endl;

    // Test A* algorithm
    AStar astar(map_loader);
    std::vector<Point> astar_path = astar.findPath(start, goal);
    testAlgorithm("A* Algorithm", astar_path, astar.getPathLength(),
                  astar.getComputeTime(), astar.pathFound());

    if (astar.pathFound()) {
        printPath(astar_path, "astar_path.txt");
    }

    // Test Dijkstra algorithm
    Dijkstra dijkstra(map_loader);
    std::vector<Point> dijkstra_path = dijkstra.findPath(start, goal);
    testAlgorithm("Dijkstra Algorithm", dijkstra_path,
                  dijkstra.getPathLength(), dijkstra.getComputeTime(),
                  dijkstra.pathFound());

    if (dijkstra.pathFound()) {
        printPath(dijkstra_path, "dijkstra_path.txt");
    }

    std::cout << "\n=== Test Complete ===\n" << std::endl;
    return 0;
}
