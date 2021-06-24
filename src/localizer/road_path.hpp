#ifndef __ROAD_PATH__
#define __ROAD_PATH__

#include "core/basic_type.hpp"

namespace dg
{

/**
* it includes start_pos and dest_pos into pts as normal path elemments
*/
class RoadPath
{
public:
    std::vector<Point2ID> pts;
};

} // End of 'dg'

#endif // End of '__ROAD_PATH__'
