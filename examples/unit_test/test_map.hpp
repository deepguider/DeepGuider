#ifndef __TEST_MAP__
#define __TEST_MAP__

#include "dg_core.hpp"

int testMapNodeInfo()
{
    dg::NodeInfo node;
    VVS_CHECK_TRUE(node.id == 0);
    VVS_CHECK_TRUE(node.x == 0);
    VVS_CHECK_TRUE(node.y == 0);
    VVS_CHECK_TRUE(node.lon == 0);
    VVS_CHECK_TRUE(node.lat == 0);
    VVS_CHECK_TRUE(node.type == 0);
    VVS_CHECK_TRUE(node.floor == 0);

    node.id = 3335;
    node.lon = 82;
    node.lat = 329;
    VVS_CHECK_TRUE(node.id == 3335);
    VVS_CHECK_TRUE(node.x == 82);
    VVS_CHECK_TRUE(node.y == 329);
    VVS_CHECK_TRUE(node.lon == 82);
    VVS_CHECK_TRUE(node.lat == 329);

    dg::LonLatID ll = node;
    VVS_CHECK_TRUE(ll.id == 3335);
    VVS_CHECK_TRUE(ll.x == 82);
    VVS_CHECK_TRUE(ll.y == 329);
    VVS_CHECK_TRUE(ll.lon == 82);
    VVS_CHECK_TRUE(ll.lat == 329);

    dg::Point2ID p = node;

    return 0;
}

#endif // End of '__TEST_MAP__'
