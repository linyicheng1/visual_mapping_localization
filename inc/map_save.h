#ifndef __MAP_SAVE_H_
#define __MAP_SAVE_H_

#include "map_point.h"
#include "frame.h"
#include <vector>
#include <string>

namespace VISUAL_MAPPING {
    class MapSaver {
    public:
        MapSaver() = default;
        ~MapSaver() = default;
        void save_map(const std::string& filename, const std::vector<std::shared_ptr<Frame>> &frames, const Map& map);
        void load_map(const std::string& filename, std::vector<std::shared_ptr<Frame>> &frames, Map& map);
    };
}


#endif //__MAP_SAVE_H_
