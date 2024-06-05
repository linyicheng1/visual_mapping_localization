#ifndef __MAP_MAPPING_H_
#define __MAP_MAPPING_H_
#include "map_point.h"
#include "frame.h"

namespace VISUAL_MAPPING {

    class Mapping {
    public:
        Mapping() = default;
        ~Mapping() = default;
        void construct_initial_map(std::vector<Frame>& frames);
        Map map;
    };
}



#endif //__MAP_MAPPING_H_
