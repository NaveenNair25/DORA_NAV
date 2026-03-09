#include "map_loader.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

MapLoader::MapLoader() : width_(0), height_(0) {}

MapLoader::~MapLoader() {}

bool MapLoader::loadMap(const std::string& yaml_file) {
    // Reset state: avoid dirty data from repeated loading
    width_ = 0;
    height_ = 0;
    map_data_.clear();
    map_info_ = MapInfo{};
    map_dir_.clear();

    if (!loadYaml(yaml_file)) {
        std::cerr << "Failed to load YAML file: " << yaml_file << std::endl;
        return false;
    }

    std::string pgm_path = map_dir_ + "/" + map_info_.image_file;
    std::cerr << "Trying to load PGM from: " << pgm_path << std::endl; // Added: print actual PGM path

    if (!loadPGM(pgm_path)) {
        std::cerr << "Failed to load PGM file: " << pgm_path << std::endl;
        return false;
    }

    // Validate threshold legality
    if (map_info_.occupied_thresh < 0.0 || map_info_.occupied_thresh > 1.0) {
        std::cerr << "Invalid occupied_thresh: " << map_info_.occupied_thresh 
                  << " (must be 0.0~1.0)" << std::endl;
        return false;
    }
    if (map_info_.free_thresh < 0.0 || map_info_.free_thresh > 1.0) {
        std::cerr << "Invalid free_thresh: " << map_info_.free_thresh 
                  << " (must be 0.0~1.0)" << std::endl;
        return false;
    }

    return true;
}

bool MapLoader::loadYaml(const std::string& yaml_file) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);

        // Added: check YAML required fields
        if (!config["image"]) {
            std::cerr << "YAML missing required field: image" << std::endl;
            return false;
        }
        if (!config["resolution"]) {
            std::cerr << "YAML missing required field: resolution" << std::endl;
            return false;
        }
        if (!config["origin"]) {
            std::cerr << "YAML missing required field: origin" << std::endl;
            return false;
        }
        if (!config["negate"]) {
            std::cerr << "YAML missing required field: negate" << std::endl;
            return false;
        }
        if (!config["occupied_thresh"]) {
            std::cerr << "YAML missing required field: occupied_thresh" << std::endl;
            return false;
        }
        if (!config["free_thresh"]) {
            std::cerr << "YAML missing required field: free_thresh" << std::endl;
            return false;
        }

        map_info_.image_file = config["image"].as<std::string>();
        map_info_.resolution = config["resolution"].as<double>();
        map_info_.origin = config["origin"].as<std::vector<double>>();
        map_info_.negate = config["negate"].as<int>();
        map_info_.occupied_thresh = config["occupied_thresh"].as<double>();
        map_info_.free_thresh = config["free_thresh"].as<double>();

        size_t last_slash = yaml_file.find_last_of("/\\");
        map_dir_ = (last_slash != std::string::npos) ?
                   yaml_file.substr(0, last_slash) : ".";

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    }
}

bool MapLoader::loadPGM(const std::string& pgm_file) {
    std::ifstream file(pgm_file, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[MapLoader] PGM file open failed (check path/permission): " << pgm_file << std::endl;
        return false;
    }

    std::string magic;
    // Read magic number and check
    if (!(file >> magic)) {
        std::cerr << "[MapLoader] PGM read magic failed (file corrupted): " << pgm_file << std::endl;
        file.close();
        return false;
    }
    if (magic != "P5") {
        std::cerr << "[MapLoader] Only P5 PGM format supported (magic: " << magic << "): " << pgm_file << std::endl;
        file.close();
        return false;
    }

    // Skip all comment lines and whitespace (PGM standard allows # comments)
    file >> std::ws;  // Skip whitespace characters
    while (file.peek() == '#') {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        file >> std::ws;  // Skip whitespace after comments
    }

    // Read width and height and check
    if (!(file >> width_ >> height_)) {
        std::cerr << "[MapLoader] PGM read width/height failed (format error): " << pgm_file << std::endl;
        file.close();
        return false;
    }
    // Validate width and height
    if (width_ <= 0 || height_ <= 0) {
        std::cerr << "[MapLoader] PGM invalid size (width/height <=0): " << width_ << "x" << height_ << std::endl;
        file.close();
        return false;
    }

    // Skip comment lines and whitespace (comments may also appear after width/height)
    file >> std::ws;
    while (file.peek() == '#') {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        file >> std::ws;
    }

    // Read max_val and check
    int max_val;
    if (!(file >> max_val)) {
        std::cerr << "[MapLoader] PGM read max_val failed: " << pgm_file << std::endl;
        file.close();
        return false;
    }
    if (max_val != 255) {
        std::cerr << "[MapLoader] PGM max_val not supported (only 255): " << max_val << std::endl;
        file.close();
        return false;
    }

    // Read newline character (separator after max_val)
    file.get();

    // Pre-allocate memory and read pixel data
    map_data_.resize(width_ * height_);
    file.read(reinterpret_cast<char*>(map_data_.data()), width_ * height_);

    // Check data read integrity
    if (!file) {
        std::cerr << "[MapLoader] PGM data size mismatch (corrupted file): " 
                  << "read " << file.gcount() << " bytes, expected " << width_ * height_ << std::endl;
        map_data_.clear();
        file.close();
        return false;
    }

    file.close();
    std::cerr << "[MapLoader] PGM loaded successfully: " << width_ << "x" << height_ << " pixels" << std::endl;
    return true;
}

bool MapLoader::isOccupied(int x, int y) const {
    if (!isValid(x, y)) return true;

    int idx = y * width_ + x;
    double value = static_cast<double>(map_data_[idx]) / 255.0;

    // Fix: handle negate parameter (original code was not effective)
    if (map_info_.negate != 0) {
        value = 1.0 - value;
    }

    return value < map_info_.occupied_thresh;
}

bool MapLoader::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

