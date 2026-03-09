#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <vector>
#include <queue>
#include <unordered_map>
#include "map_loader.h"
#include "astar.h"

class Dijkstra {
public:
    Dijkstra(const MapLoader& map);

    std::vector<Point> findPath(const Point& start, const Point& goal);
    double getPathLength() const { return path_length_; }
    double getComputeTime() const { return compute_time_; }
    bool pathFound() const { return path_found_; }

private:
    std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point, PointHash>& came_from,
        const Point& current) const;

    const MapLoader& map_;
    double path_length_;
    double compute_time_;
    bool path_found_;
};

#endif // DIJKSTRA_H
