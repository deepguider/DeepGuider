#ifndef __TEST_SIMPLE_ROAD_MAP__
#define __TEST_SIMPLE_ROAD_MAP__

#include "vvs.h"
#include "simple_road_map.hpp"

int testPoint2ID()
{
    dg::Point2ID p, p0(0, 3, 29), p1(1, dg::Point2(3, 29));
    dg::Point2ID q(0);

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

int testSimpleRoadMap(const char* filename = "test_simple_road_map.csv")
{
    // An example road map ('+' represents direction of edges)
    // 2 --+ 3 +-+ 5 +-- 6
    // +     |     +     |
    // |     +     |     +
    // 1 +-- 4     7 +-- 8

    // Test degenerate cases
    dg::SimpleRoadMap map;
    VVS_CHECK_TRUE(map.load("nothing") == false);
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_TRUE(map.save(filename) == false);

    // Build and save a map
    map.addNode(dg::Point2ID(1, 0, 0)); // ID, x, y
    map.addNode(dg::Point2ID(2, 0, 1));
    map.addNode(dg::Point2ID(3, 1, 1));
    map.addNode(dg::Point2ID(4, 1, 0));
    map.addNode(dg::Point2ID(5, 2, 1));
    map.addNode(dg::Point2ID(6, 3, 1));
    map.addNode(dg::Point2ID(7, 2, 0));
    map.addNode(dg::Point2ID(8, 3, 0));
    map.addEdge(map.getNode(dg::Point2ID(1)), map.getNode(dg::Point2ID(2))); // Automatic cost: Euclidean distance
    map.addEdge(map.getNode(dg::Point2ID(2)), map.getNode(dg::Point2ID(3)));
    map.addEdge(map.getNode(dg::Point2ID(3)), map.getNode(dg::Point2ID(4)));
    map.addEdge(map.getNode(dg::Point2ID(4)), map.getNode(dg::Point2ID(1)));
    map.addRoad(map.getNode(dg::Point2ID(3)), map.getNode(dg::Point2ID(5))); // Add a bi-directional edge
    map.addEdge(map.getNode(dg::Point2ID(6)), map.getNode(dg::Point2ID(5)));
    map.addEdge(map.getNode(dg::Point2ID(6)), map.getNode(dg::Point2ID(8)));
    map.addEdge(map.getNode(dg::Point2ID(7)), map.getNode(dg::Point2ID(5)));
    map.addEdge(map.getNode(dg::Point2ID(8)), map.getNode(dg::Point2ID(7)));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);
    VVS_CHECK_TRUE(map.save(filename));

    // Rest the map
    map.removeAll();
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_EQUAL(map.countNodes(), 0);

    // Load the map
    VVS_CHECK_TRUE(map.load(filename));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);

    // Check each node data
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(1))->data.x, 0);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(1))->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(2))->data.x, 0);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(2))->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(3))->data.x, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(3))->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(4))->data.x, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(4))->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(5))->data.x, 2);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(5))->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(6))->data.x, 3);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(6))->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(7))->data.x, 2);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(7))->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(8))->data.x, 3);
    VVS_CHECK_EQUL(map.getNode(dg::Point2ID(8))->data.y, 0);

    // Check each connectivity and cost
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(1)), map.getNode(dg::Point2ID(2))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(2)), map.getNode(dg::Point2ID(3))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(3)), map.getNode(dg::Point2ID(4))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(4)), map.getNode(dg::Point2ID(1))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(3)), map.getNode(dg::Point2ID(5))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(5)), map.getNode(dg::Point2ID(3))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(6)), map.getNode(dg::Point2ID(5))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(6)), map.getNode(dg::Point2ID(8))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(7)), map.getNode(dg::Point2ID(5))), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(map.getNode(dg::Point2ID(8)), map.getNode(dg::Point2ID(7))), 1);

    VVS_CHECK_TRUE(map.getEdgeCost(map.getNode(dg::Point2ID(1)), map.getNode(dg::Point2ID(3))) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(map.getNode(dg::Point2ID(2)), map.getNode(dg::Point2ID(1))) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(map.getNode(dg::Point2ID(5)), map.getNode(dg::Point2ID(6))) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(map.getNode(dg::Point2ID(5)), map.getNode(dg::Point2ID(7))) < 0);

    return 0;
}

#endif // End of '__TEST_SIMPLE_ROAD_MAP__'