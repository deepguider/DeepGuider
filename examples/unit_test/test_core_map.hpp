#ifndef __TEST_CORE__
#define __TEST_CORE__

#include "dg_core.hpp"

int testCoreLatLon()
{
    // Check default values
    dg::LatLon a;
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.lon == 0);

    // Check initialization
    dg::LatLon b(1, 2);
    VVS_CHECK_TRUE(b.lat == 1);
    VVS_CHECK_TRUE(b.lon == 2);

    // Check equality and inequality
    dg::LatLon c = b;
    VVS_CHECK_TRUE(b == c);
    VVS_CHECK_TRUE(b != a);

    return 0;
}

int testCorePolar2()
{
    // Check default values
    dg::Polar2 a;
    VVS_CHECK_TRUE(a.lin == 0);
    VVS_CHECK_TRUE(a.ang == 0);

    // Check initialization
    dg::Polar2 b(1, 2);
    VVS_CHECK_TRUE(b.lin == 1);
    VVS_CHECK_TRUE(b.ang == 2);

    // Check equality and inequality
    dg::Polar2 c = b;
    VVS_CHECK_TRUE(b == c);
    VVS_CHECK_TRUE(b != a);

    return 0;
}

int testCorePoint2ID()
{
    // Check default values
    dg::Point2ID a;
    VVS_CHECK_EQUL(a.id, 0);
    VVS_CHECK_EQUL(a.x, 0);
    VVS_CHECK_EQUL(a.y, 0);

    // Check initialization
    dg::Point2ID b(1, 3, 29), c(2, dg::Point2(3, 29)), d(1);
    VVS_CHECK_EQUL(b.id, 1);
    VVS_CHECK_EQUL(b.x, 3);
    VVS_CHECK_EQUL(b.y, 29);
    VVS_CHECK_EQUL(c.id, 2);
    VVS_CHECK_EQUL(c.x, 3);
    VVS_CHECK_EQUL(c.y, 29);
    VVS_CHECK_EQUL(d.id, 1);
    VVS_CHECK_EQUL(d.x, 0);
    VVS_CHECK_EQUL(d.y, 0);

    // Check equality and inequality
    VVS_CHECK_TRUE(b != c);
    VVS_CHECK_TRUE(b == d);

    return 0;
}

int testCoreNodeInfo()
{
    // Check default values
    dg::NodeInfo a;
    VVS_CHECK_TRUE(a.id == 0);
    VVS_CHECK_TRUE(a.lon == 0);
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.type == 0);
    VVS_CHECK_TRUE(a.floor == 0);

    // Check initialization
    dg::NodeInfo b(3335, dg::LatLon(82, 329), 7, 14);
    VVS_CHECK_TRUE(b.id == 3335);
    VVS_CHECK_TRUE(b.lat == 82);
    VVS_CHECK_TRUE(b.lon == 329);
    VVS_CHECK_TRUE(b.type == 7);
    VVS_CHECK_TRUE(b.floor == 14);

    // Check equality and inequality
    dg::NodeInfo c(3335);
    VVS_CHECK_TRUE(b == c);
    VVS_CHECK_TRUE(b != a);

    return 0;
}

int testCoreEdgeInfo()
{
    // Check default values
    dg::EdgeInfo a;
    VVS_CHECK_TRUE(a.length == 1);
    VVS_CHECK_TRUE(a.width == 1);
    VVS_CHECK_TRUE(a.type == 0);

    // Check initialization
    dg::EdgeInfo b(3, 2, 3);
    VVS_CHECK_TRUE(b.length == 3);
    VVS_CHECK_TRUE(b.width == 2);
    VVS_CHECK_TRUE(b.type == 3);

    return 0;
}

int testCoreMap()
{
    // An example map
    // 2 --- 3 --- 5 --- 6
    // |     |     |     |
    // |     |     |     |
    // 1 --- 4     7 --- 8

    dg::Map map;
    VVS_CHECK_TRUE(map.isEmpty());

    // Build an example map
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(1, 0, 0)) != NULL); // ID, x, y
    VVS_CHECK_TRUE(map.isEmpty() == false);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(2, 0, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(3, 1, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(4, 1, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(5, 2, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(6, 3, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(7, 2, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::NodeInfo(8, 3, 0)) != NULL);

    dg::Map::Node* node1_ptr = map.getNode(dg::NodeInfo(1));
    dg::Map::Node* node2_ptr = map.findNode(2);
    VVS_CHECK_TRUE(map.addRoad(node1_ptr, node2_ptr));              // Method #1 to add a bi-directional edge (pointer)
    VVS_CHECK_TRUE(map.addRoad(dg::NodeInfo(1), dg::NodeInfo(4)));  // Method #2 to add a bi-directional edge (NodeInfo)
    VVS_CHECK_TRUE(map.addRoad(2, 3));                              // Method #3 to add a bi-directional edge (ID)
    VVS_CHECK_TRUE(map.addRoad(3, 4));
    VVS_CHECK_TRUE(map.addRoad(3, 5));
    VVS_CHECK_TRUE(map.addRoad(5, 6));
    VVS_CHECK_TRUE(map.addRoad(5, 7));
    VVS_CHECK_TRUE(map.addRoad(6, 8));
    VVS_CHECK_TRUE(map.addRoad(7, 8));

    // Check each a data
    VVS_CHECK_TRUE(map.findNode(1)->data.lat == 0);
    VVS_CHECK_TRUE(map.findNode(1)->data.lon == 0);
    VVS_CHECK_TRUE(map.findNode(2)->data.lat == 0);
    VVS_CHECK_TRUE(map.findNode(2)->data.lon == 1);
    VVS_CHECK_TRUE(map.findNode(3)->data.lat == 1);
    VVS_CHECK_TRUE(map.findNode(3)->data.lon == 1);
    VVS_CHECK_TRUE(map.findNode(4)->data.lat == 1);
    VVS_CHECK_TRUE(map.findNode(4)->data.lon == 0);
    VVS_CHECK_TRUE(map.findNode(5)->data.lat == 2);
    VVS_CHECK_TRUE(map.findNode(5)->data.lon == 1);
    VVS_CHECK_TRUE(map.findNode(6)->data.lat == 3);
    VVS_CHECK_TRUE(map.findNode(6)->data.lon == 1);
    VVS_CHECK_TRUE(map.findNode(7)->data.lat == 2);
    VVS_CHECK_TRUE(map.findNode(7)->data.lon == 0);
    VVS_CHECK_TRUE(map.findNode(8)->data.lat == 3);
    VVS_CHECK_TRUE(map.findNode(8)->data.lon == 0);

    // Check some connectivity
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(1), dg::NodeInfo(2)));
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(2), dg::NodeInfo(1)));
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(1), dg::NodeInfo(4)));
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(4), dg::NodeInfo(1)));
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(2), dg::NodeInfo(3)));
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(3), dg::NodeInfo(2)));

    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(4), dg::NodeInfo(7)) == false);
    VVS_CHECK_TRUE(map.isConnected(dg::NodeInfo(7), dg::NodeInfo(4)) == false);

    // Check some 'length' in 'dg::EdgeCost'
    VVS_CHECK_TRUE(map.findEdge(1, 2)->cost.length == 1);
    VVS_CHECK_TRUE(map.findEdge(2, 1)->cost.length == 1);
    VVS_CHECK_TRUE(map.findEdge(1, 4)->cost.length == 1);
    VVS_CHECK_TRUE(map.findEdge(4, 1)->cost.length == 1);
    VVS_CHECK_TRUE(map.findEdge(2, 3)->cost.length == 1);
    VVS_CHECK_TRUE(map.findEdge(3, 2)->cost.length == 1);

    VVS_CHECK_TRUE(map.findEdge(4, 7) == NULL);
    VVS_CHECK_TRUE(map.getEdgeCost(dg::NodeInfo(7), dg::NodeInfo(4)).length < 0);

    // Clear the map
    VVS_CHECK_TRUE(map.removeAll());
    VVS_CHECK_TRUE(map.isEmpty());

    return 0;
}

#endif // End of '__TEST_CORE__'
