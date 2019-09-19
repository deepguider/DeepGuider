#ifndef __TEST_CORE__
#define __TEST_CORE__

#include "dg_core.hpp"

int testCoreLonLat()
{
    // Check default values
    dg::LonLat a;
    VVS_CHECK_TRUE(a.lon == 0);
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.x == 0);
    VVS_CHECK_TRUE(a.y == 0);

    // Check initialization
    dg::LonLat b(1, 2);
    VVS_CHECK_TRUE(b.lon == 1);
    VVS_CHECK_TRUE(b.lat == 2);
    VVS_CHECK_TRUE(b.x == 1);
    VVS_CHECK_TRUE(b.y == 2);

    // Check reference to 'x' and 'y'
    a.lon = 1;
    a.lat = 2;
    VVS_CHECK_TRUE(a.lon == 1);
    VVS_CHECK_TRUE(a.lat == 2);
    VVS_CHECK_TRUE(a.x == 1);
    VVS_CHECK_TRUE(a.y == 2);
    VVS_CHECK_TRUE(a == b);

    // Check assignment with 'dg::Point2'
    b = dg::Point2(3, 4);
    VVS_CHECK_TRUE(b.lon == 3);
    VVS_CHECK_TRUE(b.lat == 4);
    VVS_CHECK_TRUE(b.x == 3);
    VVS_CHECK_TRUE(b.y == 4);
    VVS_CHECK_TRUE(a != b);

    // Check arithmetic operations
    a = 2 * b - b;
    VVS_CHECK_TRUE(a.lon == 3);
    VVS_CHECK_TRUE(a.lat == 4);
    VVS_CHECK_TRUE(a.x == 3);
    VVS_CHECK_TRUE(a.y == 4);
    VVS_CHECK_TRUE(a == b);

    return 0;
}

int testCorePoint2ID()
{
    dg::Point2ID p, p0(1, 3, 29), p1(2, dg::Point2(3, 29));
    dg::Point2ID q(1);

    // Check default values
    VVS_CHECK_EQUL(p.id, 0);
    VVS_CHECK_EQUL(p.x, 0);
    VVS_CHECK_EQUL(p.y, 0);

    // Check equality and inequality
    VVS_CHECK_TRUE(p != q);
    VVS_CHECK_TRUE(p0 == q);
    VVS_CHECK_TRUE(p0 != p1);

    return 0;
}

int testCoreNodeInfo()
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

    // Check down casting to 'dg::LonLatID'
    dg::LonLatID ll = node;
    VVS_CHECK_TRUE(ll.id == 3335);
    VVS_CHECK_TRUE(ll.x == 82);
    VVS_CHECK_TRUE(ll.y == 329);
    VVS_CHECK_TRUE(ll.lon == 82);
    VVS_CHECK_TRUE(ll.lat == 329);

    // Check down casting to 'dg::Point2ID'
    dg::Point2ID pt = node;
    VVS_CHECK_TRUE(pt.id == 3335);
    VVS_CHECK_TRUE(pt.x == 82);
    VVS_CHECK_TRUE(pt.y == 329);

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

    // Check each node data
    VVS_CHECK_TRUE(map.findNode(1)->data.x == 0);
    VVS_CHECK_TRUE(map.findNode(1)->data.y == 0);
    VVS_CHECK_TRUE(map.findNode(2)->data.x == 0);
    VVS_CHECK_TRUE(map.findNode(2)->data.y == 1);
    VVS_CHECK_TRUE(map.findNode(3)->data.x == 1);
    VVS_CHECK_TRUE(map.findNode(3)->data.y == 1);
    VVS_CHECK_TRUE(map.findNode(4)->data.x == 1);
    VVS_CHECK_TRUE(map.findNode(4)->data.y == 0);
    VVS_CHECK_TRUE(map.findNode(5)->data.x == 2);
    VVS_CHECK_TRUE(map.findNode(5)->data.y == 1);
    VVS_CHECK_TRUE(map.findNode(6)->data.x == 3);
    VVS_CHECK_TRUE(map.findNode(6)->data.y == 1);
    VVS_CHECK_TRUE(map.findNode(7)->data.x == 2);
    VVS_CHECK_TRUE(map.findNode(7)->data.y == 0);
    VVS_CHECK_TRUE(map.findNode(8)->data.x == 3);
    VVS_CHECK_TRUE(map.findNode(8)->data.y == 0);

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
