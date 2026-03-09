#include "astar.h"
#include <cmath>
#include <chrono>
#include <algorithm>

struct Node {
    Point point;
    double g_cost;
    double f_cost;

    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

AStar::AStar(const MapLoader& map)
    : map_(map), path_length_(0.0), compute_time_(0.0), path_found_(false) {}

double AStar::heuristic(const Point& a, const Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<Point> AStar::findPath(const Point& start, const Point& goal) {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<Point, Point, PointHash> came_from;
    std::unordered_map<Point, double, PointHash> g_score;

    g_score[start] = 0.0;
    open_set.push({start, 0.0, heuristic(start, goal)});

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (current.point == goal) {
            path_found_ = true;
            auto path = reconstructPath(came_from, current.point);
            path_length_ = g_score[goal];

            auto end_time = std::chrono::high_resolution_clock::now();
            compute_time_ = std::chrono::duration<double, std::milli>(
                end_time - start_time).count();
            return path;
        }

        for (int i = 0; i < 8; ++i) {
            Point neighbor(current.point.x + dx[i], current.point.y + dy[i]);

            if (!map_.isValid(neighbor.x, neighbor.y) ||
                map_.isOccupied(neighbor.x, neighbor.y)) {
                continue;
            }

            double tentative_g = g_score[current.point] + costs[i];

            if (g_score.find(neighbor) == g_score.end() ||
                tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current.point;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal);
                open_set.push({neighbor, tentative_g, f});
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    compute_time_ = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    path_found_ = false;
    return std::vector<Point>();
}

std::vector<Point> AStar::reconstructPath(
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
