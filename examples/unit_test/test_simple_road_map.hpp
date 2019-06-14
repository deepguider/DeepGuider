#ifndef __TEST_SIMPLE_ROAD_MAP__
#define __TEST_SIMPLE_ROAD_MAP__

#include "vvs.h"
#include "simple_road_map.hpp"

int testPointID()
{
    dg::PointID p, p0(0, 3, 29), p1(1, dg::Point(3, 29));
    dg::PointID q(0);

    // Check default values
    VVS_CHECK_EQUL(p.id, -1);
    VVS_CHECK_EQUL(p.x, 0);
    VVS_CHECK_EQUL(p.y, 0);

    // Check equality and inequality
    VVS_CHECK_TRUE(p != q);
    VVS_CHECK_TRUE(p0 == q);
    VVS_CHECK_TRUE(p0 != p1);
    return 0;
}

#endif // End of '__TEST_SIMPLE_ROAD_MAP__'