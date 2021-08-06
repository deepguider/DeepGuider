#ifndef __TEST_CORE_TYPE__
#define __TEST_CORE_TYPE__

#include "dg_core.hpp"
#include "utils/vvs.h"

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
    VVS_CHECK_TRUE(a.x == 0);
    VVS_CHECK_TRUE(a.y == 0);
    VVS_CHECK_TRUE(a.type == 0);
    VVS_CHECK_TRUE(a.floor == 0);
    VVS_CHECK_TRUE(a.edge_ids.empty());

    // Check initialization
    dg::Node b(3335, dg::Point2(82, 329), dg::Node::NODE_ESCALATOR, 17);
    VVS_CHECK_TRUE(b.id == 3335);
    VVS_CHECK_TRUE(b.x == 82);
    VVS_CHECK_TRUE(b.y == 329);
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
    VVS_CHECK_TRUE(a.length == -1);
    VVS_CHECK_TRUE(a.type == 0);
    VVS_CHECK_TRUE(a.directed == false);
    VVS_CHECK_TRUE(a.node_id1 == 0);
    VVS_CHECK_TRUE(a.node_id2 == 0);

    // Check initialization
    dg::Edge b(3335, dg::Edge::EDGE_ESCALATOR, 3, 29, 4, true);
    VVS_CHECK_TRUE(b.id == 3335);
    VVS_CHECK_TRUE(b.type == dg::Edge::EDGE_ESCALATOR);
    VVS_CHECK_TRUE(b.node_id1 == 3);
    VVS_CHECK_TRUE(b.node_id2 == 29);
    VVS_CHECK_TRUE(b.length == 4);
    VVS_CHECK_TRUE(b.directed == true);

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
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0))); // Given: node ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)));

    VVS_CHECK_TRUE(map.addEdge(dg::Edge(9, 0, 1, 2, 12))); // Given: edge ID, type, node1, node2, length
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(10, 0, 1, 4, 14)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(11, 0, 2, 3, 23)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(12, 0, 3, 4, 34)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(13, 0, 3, 5, 35)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(14, 0, 5, 6, 56)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(15, 0, 5, 7, 57)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(16, 0, 6, 8, 68), true));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(17, 0, 7, 8)));

    // Check each a data
    VVS_CHECK_TRUE(map.getNode(1)->x == 0);
    VVS_CHECK_TRUE(map.getNode(1)->y == 0);
    VVS_CHECK_TRUE(map.getNode(2)->x == 0);
    VVS_CHECK_TRUE(map.getNode(2)->y == 1);
    VVS_CHECK_TRUE(map.getNode(3)->x == 1);
    VVS_CHECK_TRUE(map.getNode(3)->y == 1);
    VVS_CHECK_TRUE(map.getNode(4)->x == 1);
    VVS_CHECK_TRUE(map.getNode(4)->y == 0);
    VVS_CHECK_TRUE(map.getNode(5)->x == 2);
    VVS_CHECK_TRUE(map.getNode(5)->y == 1);
    VVS_CHECK_TRUE(map.getNode(6)->x == 3);
    VVS_CHECK_TRUE(map.getNode(6)->y == 1);
    VVS_CHECK_TRUE(map.getNode(7)->x == 2);
    VVS_CHECK_TRUE(map.getNode(7)->y == 0);
    VVS_CHECK_TRUE(map.getNode(8)->x == 3);
    VVS_CHECK_TRUE(map.getNode(8)->y == 0);

    // Check some connectivity
    VVS_CHECK_TRUE(map.getEdge(1, 2) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(2, 1) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(1, 4) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(4, 1) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(2, 3) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(3, 2) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(4, 7) == nullptr);
    VVS_CHECK_TRUE(map.getEdge(7, 4) == nullptr);

    // Check some 'length' in 'dg::EdgeCost'
    VVS_CHECK_TRUE(map.getEdge(10)->length == 14);
    VVS_CHECK_TRUE(map.getEdge(11)->length == 23);
    VVS_CHECK_TRUE(map.getEdge(12)->length == 34);
    VVS_CHECK_TRUE(map.getEdge(16)->length == 1);
    VVS_CHECK_TRUE(map.getEdge(17)->length == 1);

    VVS_CHECK_TRUE(map.getEdge(21) == nullptr);
    VVS_CHECK_TRUE(map.getEdge(41) == nullptr);
    VVS_CHECK_TRUE(map.getEdge(32) == nullptr);

    return 0;
}

int testCorePath()
{
    // Build an example map
    dg::Map map;
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0))); // Given: node ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)));

    VVS_CHECK_TRUE(map.addEdge(dg::Edge(9, 0, 1, 2, 12))); // Given: edge ID, type, node1, node2, length
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(10, 0, 1, 4, 14)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(11, 0, 2, 3, 23)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(12, 0, 3, 4, 34)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(13, 0, 3, 5, 35)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(14, 0, 5, 6, 56)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(15, 0, 5, 7, 57)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(16, 0, 6, 8, 68), true));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(17, 0, 7, 8)));

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
        dg::Node* from = map.getNode(ids[i]);
        VVS_CHECK_TRUE(from != nullptr);
        dg::Edge* edge = map.getEdge(ids[i], ids[i + 1]);
        VVS_CHECK_TRUE(edge != nullptr);
        path.pts.push_back(dg::PathNode(*from, edge->id));
    }
    path.pts.push_back(dg::PathNode(map.nodes.back(), 0));
    VVS_CHECK_TRUE(ids.size() == path.pts.size());

    return 0;
}

#endif // End of '__TEST_CORE_TYPE__'
