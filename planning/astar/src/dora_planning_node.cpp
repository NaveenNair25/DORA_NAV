// DORA Path Planning Node
extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <nlohmann/json.hpp>

#include "map_loader.h"
#include "astar.h"
#include "dijkstra.h"

using json = nlohmann::json;
using namespace std;

// Global variables
MapLoader map_loader;
string map_yaml_path;
string algorithm_type = "astar";
bool map_loaded = false;

Point start_point;
Point goal_point;
bool start_received = false;
bool goal_received = false;

// Function declarations
int run(void *dora_context);
void process_start_point(char *data, size_t data_len);
void process_goal_point(char *data, size_t data_len);
void plan_and_publish_path(void *dora_context);

int main()
{
    cout << "=== DORA Path Planning Node ===" << endl;

    // Load map path from environment variable
    const char* map_path_env = std::getenv("MAP_YAML_PATH");
    if (map_path_env) {
        map_yaml_path = string(map_path_env);
    } else {
        map_yaml_path = "map/test_map.yaml";
    }

    const char* algo_env = std::getenv("ALGORITHM");
    if (algo_env) {
        algorithm_type = string(algo_env);
    }

    cout << "Map path: " << map_yaml_path << endl;
    cout << "Algorithm: " << algorithm_type << endl;

    // Load map
    if (map_loader.loadMap(map_yaml_path)) {
        map_loaded = true;
        cout << "Map loaded successfully!" << endl;
        cout << "Map size: " << map_loader.getWidth() << "x"
             << map_loader.getHeight() << endl;
    } else {
        cerr << "Failed to load map!" << endl;
        return -1;
    }

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    cout << "Exit Path Planning Node..." << endl;
    return ret;
}

int run(void *dora_context)
{
    while(true)
    {
        void *event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[path planning node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);

            if (strncmp(id, "start_point", 11) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_start_point(data, data_len);

                // If both start and goal received, plan path
                if (start_received && goal_received) {
                    plan_and_publish_path(dora_context);
                    start_received = false;
                    goal_received = false;
                }
            }
            else if (strncmp(id, "goal_point", 10) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                process_goal_point(data, data_len);

                // If both start and goal received, plan path
                if (start_received && goal_received) {
                    plan_and_publish_path(dora_context);
                    start_received = false;
                    goal_received = false;
                }
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[path planning node] received stop event\n");
            break;
        }
        else
        {
            printf("[path planning node] received unexpected event: %d\n", ty);
        }
        free_dora_event(event);
    }
    return 0;
}

void process_start_point(char *data, size_t data_len)
{
    string data_str(data, data_len);
    try {
        json j = json::parse(data_str);
        start_point.x = j["x"];
        start_point.y = j["y"];
        start_received = true;
        cout << "Received start point: (" << start_point.x << ", "
             << start_point.y << ")" << endl;
    } catch (const json::parse_error& e) {
        cerr << "JSON parse error in start_point: " << e.what() << endl;
    }
}

void process_goal_point(char *data, size_t data_len)
{
    string data_str(data, data_len);
    try {
        json j = json::parse(data_str);
        goal_point.x = j["x"];
        goal_point.y = j["y"];
        goal_received = true;
        cout << "Received goal point: (" << goal_point.x << ", "
             << goal_point.y << ")" << endl;
    } catch (const json::parse_error& e) {
        cerr << "JSON parse error in goal_point: " << e.what() << endl;
    }
}

void plan_and_publish_path(void *dora_context)
{
    if (!map_loaded) {
        cerr << "Map not loaded!" << endl;
        return;
    }

    cout << "\n=== Planning path ===" << endl;
    cout << "Start: (" << start_point.x << ", " << start_point.y << ")" << endl;
    cout << "Goal: (" << goal_point.x << ", " << goal_point.y << ")" << endl;

    vector<Point> path;
    double path_length = 0.0;
    double compute_time = 0.0;
    bool path_found = false;

    // Execute planning algorithm
    if (algorithm_type == "astar") {
        AStar planner(map_loader);
        path = planner.findPath(start_point, goal_point);
        path_length = planner.getPathLength();
        compute_time = planner.getComputeTime();
        path_found = planner.pathFound();
    } else if (algorithm_type == "dijkstra") {
        Dijkstra planner(map_loader);
        path = planner.findPath(start_point, goal_point);
        path_length = planner.getPathLength();
        compute_time = planner.getComputeTime();
        path_found = planner.pathFound();
    }

    // Print results
    cout << "Algorithm: " << algorithm_type << endl;
    cout << "Path found: " << (path_found ? "YES" : "NO") << endl;
    if (path_found) {
        cout << "Path length: " << path_length << " grid units" << endl;
        cout << "Waypoints: " << path.size() << endl;
        cout << "Compute time: " << compute_time << " ms" << endl;
    }

    // Create JSON output
    json j_path;
    j_path["algorithm"] = algorithm_type;
    j_path["path_found"] = path_found;
    j_path["path_length"] = path_length;
    j_path["compute_time_ms"] = compute_time;
    j_path["num_waypoints"] = path.size();

    json waypoints_array = json::array();
    for (const auto& point : path) {
        json waypoint;
        waypoint["x"] = point.x;
        waypoint["y"] = point.y;
        waypoints_array.push_back(waypoint);
    }
    j_path["waypoints"] = waypoints_array;

    // Serialize and send
    string json_string = j_path.dump();
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());

    string out_id = "path";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(),
                                   c_json_string, strlen(c_json_string));

    if (result != 0) {
        cerr << "Failed to send path output" << endl;
    } else {
        cout << "Path published successfully!" << endl;
    }

    delete[] c_json_string;
}
