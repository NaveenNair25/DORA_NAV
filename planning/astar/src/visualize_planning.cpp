#include "map_loader.h"
#include "astar.h"
#include "dijkstra.h"
#include <iostream>
#include <cstdint>

// Rerun C++ SDK headers
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

// Helper function to draw a pixel on rgb_data
void drawPixel(std::vector<uint8_t> &rgb_data, uint32_t width, uint32_t height,
               int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x >= 0 && x < static_cast<int>(width) && y >= 0 && y < static_cast<int>(height))
    {
        int idx = (y * width + x) * 3;
        rgb_data[idx] = r;
        rgb_data[idx + 1] = g;
        rgb_data[idx + 2] = b;
    }
}

// Helper function to draw a circle
void drawCircle(std::vector<uint8_t> &rgb_data, uint32_t width, uint32_t height,
                int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b)
{
    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            if (dx * dx + dy * dy <= radius * radius)
            {
                drawPixel(rgb_data, width, height, cx + dx, cy + dy, r, g, b);
            }
        }
    }
}

// Function to draw a path on the map
void drawPathOnMap(std::vector<uint8_t> &rgb_data, uint32_t width, uint32_t height,
                   const std::vector<Point> &path, uint8_t r, uint8_t g, uint8_t b,
                   const std::string &path_name)
{
    std::cout << "Drawing " << path_name << " path..." << std::endl;
    for (const auto &p : path)
    {
        drawPixel(rgb_data, width, height, p.x, p.y, r, g, b);
    }
}

void visualizeWithRerun(const MapLoader &map_loader,
                        const std::vector<Point> &astar_path,
                        const std::vector<Point> &dijkstra_path)
{
    std::cout << "\n=== Rerun Visualization ===\n";
    std::cout << "Note: This requires rerun-sdk to be installed.\n";
    std::cout << "Install: pip install rerun-sdk\n";
    std::cout << "Then rebuild with rerun C++ SDK linked.\n\n";

    const auto rec = rerun::RecordingStream("path_planning");
    auto spawn_result = rec.spawn();
    if (spawn_result.is_err())
    {
        std::cerr << "Failed to start rerun: " << std::endl;
        return;
    }

    // Log map as image
    int width_int = map_loader.getWidth();
    int height_int = map_loader.getHeight();
    const auto &data = map_loader.getData();

    const uint32_t width = static_cast<uint32_t>(width_int);
    const uint32_t height = static_cast<uint32_t>(height_int);

    std::vector<uint8_t> rgb_data(width * height * 3);
    for (uint32_t i = 0; i < width * height; ++i)
    {
        rgb_data[i * 3] = data[i];
        rgb_data[i * 3 + 1] = data[i];
        rgb_data[i * 3 + 2] = data[i];
    }

    // Draw A* path in blue
    drawPathOnMap(rgb_data, width, height, astar_path, 0, 0, 255, "A*");

    // Draw Dijkstra path in cyan
    drawPathOnMap(rgb_data, width, height, dijkstra_path, 0, 255, 255, "Dijkstra");

    // Draw start point as red circle
    if (!astar_path.empty())
    {
        const auto &start = astar_path.front();
        std::cout << "Drawing start point at (" << start.x << ", " << start.y << ") in red" << std::endl;
        drawCircle(rgb_data, width, height, start.x, start.y, 5, 255, 0, 0);
    }

    // Draw end point as green circle
    if (!astar_path.empty())
    {
        const auto &goal = astar_path.back();
        std::cout << "Drawing goal point at (" << goal.x << ", " << goal.y << ") in green" << std::endl;
        drawCircle(rgb_data, width, height, goal.x, goal.y, 5, 0, 255, 0);
    }

    // Log the map with trajectories drawn on it
    rec.log("map_with_trajectories", rerun::Image::from_rgb24(rgb_data, {width, height}));

    // Also log the original map for comparison
    std::vector<uint8_t> rgb_data_original(width * height * 3);
    for (uint32_t i = 0; i < width * height; ++i)
    {
        rgb_data_original[i * 3] = data[i];
        rgb_data_original[i * 3 + 1] = data[i];
        rgb_data_original[i * 3 + 2] = data[i];
    }
    rec.log("map", rerun::Image::from_rgb24(rgb_data_original, {width, height}));

    // Log A* path
    std::vector<rerun::Position3D> astar_points;
    std::cout << "A* path size:" << astar_path.size() << std::endl;
    for (const auto &p : astar_path)
    {
        astar_points.push_back({static_cast<float>(p.x),
                                static_cast<float>(p.y), 1.0f});
    }
    size_t NUM_POINTS = astar_path.size();
    std::vector<rerun::Color> colors1;
    rerun::demo::color_spiral(NUM_POINTS, 2.0f, 0.02f, 0.0f, 0.1f, astar_points, colors1);
    rec.log("astar_path",
            rerun::Points3D(astar_points).with_colors(colors1).with_radii({0.08f}));

    // Log Dijkstra path
    std::vector<rerun::Position3D> dijkstra_points;
    for (const auto &p : dijkstra_path)
    {
        dijkstra_points.push_back({static_cast<float>(p.x),
                                   static_cast<float>(p.y), 1.0f});
    }
    NUM_POINTS = dijkstra_points.size();
    std::vector<rerun::Color> colors2;
    rerun::demo::color_spiral(NUM_POINTS, 2.0f, 0.02f, 0.0f, 0.1f, dijkstra_points, colors2);
    rec.log("dijkstra_path",
            rerun::Points3D(dijkstra_points).with_colors(colors2).with_radii({0.08f}));

    std::cout << "Rerun visualization started! Check the rerun window.\n";
}

int main(int argc, char **argv)
{
    std::cout << "=== Path Planning with Visualization ===\n"
              << std::endl;

    MapLoader map_loader;
    std::string map_file = "../map/test_map.yaml";

    std::cout << "Trying to load map from: " << map_file << std::endl;
    bool load_success = map_loader.loadMap(map_file);
    std::cout << "Map load success: " << (load_success ? "YES" : "NO") << std::endl;

    if (!load_success)
    {
        std::cerr << "Failed to load map!" << std::endl;
        return 1;
    }

    int map_width = map_loader.getWidth();
    int map_height = map_loader.getHeight();
    std::cout << "Map dimensions - Width: " << map_width << ", Height: " << map_height << std::endl;
    if (map_width <= 0 || map_height <= 0)
    {
        std::cerr << "Error: Map width/height is zero or negative!" << std::endl;
        return 1;
    }

    Point start(200, 193);
    Point goal(255, 304);

    // Check if start or goal is on an obstacle
    if (map_loader.isOccupied(start.x, start.y))
    {
        std::cerr << "Error: Start point (" << start.x << ", " << start.y
                  << ") is on an obstacle!" << std::endl;
        return 1;
    }
    if (map_loader.isOccupied(goal.x, goal.y))
    {
        std::cerr << "Error: Goal point (" << goal.x << ", " << goal.y
                  << ") is on an obstacle!" << std::endl;
        return 1;
    }

    std::cout << "Start point: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal point: (" << goal.x << ", " << goal.y << ")" << std::endl;

    AStar astar(map_loader);
    std::vector<Point> astar_path = astar.findPath(start, goal);

    Dijkstra dijkstra(map_loader);
    std::vector<Point> dijkstra_path = dijkstra.findPath(start, goal);

    visualizeWithRerun(map_loader, astar_path, dijkstra_path);

    return 0;
}
