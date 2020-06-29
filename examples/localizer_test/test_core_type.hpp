#ifndef __TEST_CORE_TYPE__
#define __TEST_CORE_TYPE__

#include "vvs.h"
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

int testCoreNode()
{
    // Check default values
    dg::Node a;
    VVS_CHECK_TRUE(a.id == 0);
    VVS_CHECK_TRUE(a.lon == 0);
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.type == 0);
    VVS_CHECK_TRUE(a.floor == 0);
    VVS_CHECK_TRUE(a.edge_ids.empty());

    // Check initialization
    dg::Node b(3335, dg::LatLon(82, 329), dg::Node::NODE_ESCALATOR, 17);
    VVS_CHECK_TRUE(b.id == 3335);
    VVS_CHECK_TRUE(b.lat == 82);
    VVS_CHECK_TRUE(b.lon == 329);
    VVS_CHECK_TRUE(b.type == dg::Node::NODE_ESCALATOR);
    VVS_CHECK_TRUE(b.floor == 17);
    VVS_CHECK_TRUE(b.edge_ids.empty());

    // Check equality and inequality
    dg::Node c(3335);
    VVS_CHECK_TRUE(b == c);
    VVS_CHECK_TRUE(b != a);

    return 0;
}

int testCoreEdge()
{
    // Check default values
    dg::Edge a;
    VVS_CHECK_TRUE(a.id == 0);
    VVS_CHECK_TRUE(a.length == 1);
    VVS_CHECK_TRUE(a.type == 0);
    VVS_CHECK_TRUE(a.directed == false);
    VVS_CHECK_TRUE(a.node_id1 == 0);
    VVS_CHECK_TRUE(a.node_id2 == 0);

    // Check initialization
    dg::Edge b(3335, 3, dg::Edge::EDGE_ESCALATOR, true, 3, 29);
    VVS_CHECK_TRUE(b.id == 3335);
    VVS_CHECK_TRUE(b.length == 3);
    VVS_CHECK_TRUE(b.type == dg::Edge::EDGE_ESCALATOR);
    VVS_CHECK_TRUE(b.directed == true);
    VVS_CHECK_TRUE(b.node_id1 == 3);
    VVS_CHECK_TRUE(b.node_id2 == 29);

    // Check equality and inequality
    dg::Edge c(3335);
    VVS_CHECK_TRUE(b == c);
    VVS_CHECK_TRUE(b != a);

    return 0;
}

int testCoreMap()
{
    // An example map
    // 2 --- 3 --- 5 --- 6
    // |     |     |     |
    // |     |     |     |
    // 1 --- 4     7 --- 8

    // Build an example map
    dg::Map map;
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0)) >= 0); // Given: node ID, latitude, longitude
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)) >= 0);

    VVS_CHECK_TRUE(map.addEdge(1, 2, dg::Edge(12)) >= 0); // Given: node1, node2, edge ID
    VVS_CHECK_TRUE(map.addEdge(1, 4, dg::Edge(14)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(2, 3, dg::Edge(23)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(3, 4, dg::Edge(34)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(3, 5, dg::Edge(35)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(5, 6, dg::Edge(56)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(5, 7, dg::Edge(57)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(6, 8, dg::Edge(68)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(7, 8, dg::Edge(78)) >= 0);

    // Check each a data
    VVS_CHECK_TRUE(map.findNode(1)->lat == 0);
    VVS_CHECK_TRUE(map.findNode(1)->lon == 0);
    VVS_CHECK_TRUE(map.findNode(2)->lat == 0);
    VVS_CHECK_TRUE(map.findNode(2)->lon == 1);
    VVS_CHECK_TRUE(map.findNode(3)->lat == 1);
    VVS_CHECK_TRUE(map.findNode(3)->lon == 1);
    VVS_CHECK_TRUE(map.findNode(4)->lat == 1);
    VVS_CHECK_TRUE(map.findNode(4)->lon == 0);
    VVS_CHECK_TRUE(map.findNode(5)->lat == 2);
    VVS_CHECK_TRUE(map.findNode(5)->lon == 1);
    VVS_CHECK_TRUE(map.findNode(6)->lat == 3);
    VVS_CHECK_TRUE(map.findNode(6)->lon == 1);
    VVS_CHECK_TRUE(map.findNode(7)->lat == 2);
    VVS_CHECK_TRUE(map.findNode(7)->lon == 0);
    VVS_CHECK_TRUE(map.findNode(8)->lat == 3);
    VVS_CHECK_TRUE(map.findNode(8)->lon == 0);

    // Check some connectivity
    VVS_CHECK_TRUE(map.findEdge(1, 2) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(2, 1) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(1, 4) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(4, 1) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(2, 3) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(3, 2) != nullptr);
    VVS_CHECK_TRUE(map.findEdge(4, 7) == nullptr);
    VVS_CHECK_TRUE(map.findEdge(7, 4) == nullptr);

    // Check some 'length' in 'dg::EdgeCost'
    VVS_CHECK_TRUE(map.findEdge(12)->length == 1);
    VVS_CHECK_TRUE(map.findEdge(14)->length == 1);
    VVS_CHECK_TRUE(map.findEdge(23)->length == 1);

    VVS_CHECK_TRUE(map.findEdge(21) == nullptr);
    VVS_CHECK_TRUE(map.findEdge(41) == nullptr);
    VVS_CHECK_TRUE(map.findEdge(32) == nullptr);

    return 0;
}

int testCorePath()
{
    // Build an example map
    dg::Map map;
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0)) >= 0); // Given: node ID, latitude, longitude
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)) >= 0);
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)) >= 0);

    VVS_CHECK_TRUE(map.addEdge(1, 2, dg::Edge(12)) >= 0); // Given: node1, node2, edge ID
    VVS_CHECK_TRUE(map.addEdge(1, 4, dg::Edge(14)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(2, 3, dg::Edge(23)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(3, 4, dg::Edge(34)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(3, 5, dg::Edge(35)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(5, 6, dg::Edge(56)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(5, 7, dg::Edge(57)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(6, 8, dg::Edge(68)) >= 0);
    VVS_CHECK_TRUE(map.addEdge(7, 8, dg::Edge(78)) >= 0);

    // An example path
    // 2 --- 3 --- 5 --- 6
    // |                 |
    // |                 |
    // 1     4     7     8

    // Build a path
    std::vector<dg::ID> ids = { 1, 2, 3, 5, 6, 8 };
    dg::Path path;
    for (size_t i = 0; i < ids.size() - 1; i++)
    {
        dg::Node* from = map.findNode(ids[i]);
        VVS_CHECK_TRUE(from != nullptr);
        dg::Edge* edge = map.findEdge(ids[i], ids[i + 1]);
        VVS_CHECK_TRUE(edge != nullptr);
        path.pts.push_back(dg::PathElement(from->id, edge->id));
    }
    path.pts.push_back(dg::PathElement(map.nodes.back().id, 0));
    VVS_CHECK_TRUE(ids.size() == path.pts.size());

    return 0;
}

#endif // End of '__TEST_CORE_TYPE__'
