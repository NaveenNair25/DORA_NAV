#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <string>
#include <vector>
#include <cstdint>  // Add header dependency for uint8_t

struct MapInfo {
    std::string image_file;
    double resolution;
    std::vector<double> origin;
    int negate;
    double occupied_thresh;
    double free_thresh;
};

class MapLoader {
public:
    MapLoader();
    ~MapLoader();

    bool loadMap(const std::string& yaml_file);
    bool isOccupied(int x, int y) const;
    bool isValid(int x, int y) const;

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    const std::vector<uint8_t>& getData() const { return map_data_; }
    const MapInfo& getMapInfo() const { return map_info_; }

private:
    bool loadYaml(const std::string& yaml_file);
    bool loadPGM(const std::string& pgm_file);

    int width_;
    int height_;
    std::vector<uint8_t> map_data_;
    MapInfo map_info_;
    std::string map_dir_;
};

#endif // MAP_LOADER_H

