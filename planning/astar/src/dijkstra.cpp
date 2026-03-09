#include "dijkstra.h"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <limits>

struct DijkstraNode {
    Point point;
    double cost;

    bool operator>(const DijkstraNode& other) const {
        return cost > other.cost;
    }
};

Dijkstra::Dijkstra(const MapLoader& map)
    : map_(map), path_length_(0.0), compute_time_(0.0), path_found_(false) {}

std::vector<Point> Dijkstra::findPath(const Point& start, const Point& goal) {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>,
                        std::greater<DijkstraNode>> open_set;
    std::unordered_map<Point, Point, PointHash> came_from;
    std::unordered_map<Point, double, PointHash> cost_so_far;

    cost_so_far[start] = 0.0;
    open_set.push({start, 0.0});

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!open_set.empty()) {
        DijkstraNode current = open_set.top();
        open_set.pop();

        if (current.point == goal) {
            path_found_ = true;
            auto path = reconstructPath(came_from, current.point);
            path_length_ = cost_so_far[goal];

            auto end_time = std::chrono::high_resolution_clock::now();
            compute_time_ = std::chrono::duration<double, std::milli>(
                end_time - start_time).count();
            return path;
        }

        if (current.cost > cost_so_far[current.point]) {
            continue;
        }

        for (int i = 0; i < 8; ++i) {
            Point neighbor(current.point.x + dx[i], current.point.y + dy[i]);

            if (!map_.isValid(neighbor.x, neighbor.y) ||
                map_.isOccupied(neighbor.x, neighbor.y)) {
                continue;
            }

            double new_cost = cost_so_far[current.point] + costs[i];

            if (cost_so_far.find(neighbor) == cost_so_far.end() ||
                new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                came_from[neighbor] = current.point;
                open_set.push({neighbor, new_cost});
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    compute_time_ = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    path_found_ = false;
    return std::vector<Point>();
}

std::vector<Point> Dijkstra::reconstructPath(
    const std::unordered_map<Point, Point, PointHash>& came_from,
    const Point& current) const {
    std::vector<Point> path;
    Point temp = current;
    path.push_back(temp);

    while (came_from.find(temp) != came_from.end()) {
        temp = came_from.at(temp);
        path.push_back(temp);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
