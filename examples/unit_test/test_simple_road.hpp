#ifndef __TEST_SIMPLE_ROAD_MAP__
#define __TEST_SIMPLE_ROAD_MAP__

#include "vvs.h"
#include "dg_core.hpp"

int testSimplePoint2ID()
{
    dg::Point2ID p, p0(0, 3, 29), p1(1, dg::Point2(3, 29));
    dg::Point2ID q(0);

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
    map.addEdge(dg::Point2ID(1), dg::Point2ID(2));
    map.addEdge(dg::Point2ID(2), dg::Point2ID(3));
    map.addEdge(dg::Point2ID(3), dg::Point2ID(4));
    map.addEdge(dg::Point2ID(4), dg::Point2ID(1));
    map.addRoad(dg::Point2ID(3), dg::Point2ID(5)); // Add a bi-directional edge
    map.addEdge(dg::Point2ID(6), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(6), dg::Point2ID(8));
    map.addEdge(dg::Point2ID(7), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(8), dg::Point2ID(7));
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
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(1), dg::Point2ID(2)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(2), dg::Point2ID(3)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(3), dg::Point2ID(4)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(4), dg::Point2ID(1)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(3), dg::Point2ID(5)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(5), dg::Point2ID(3)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(6), dg::Point2ID(5)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(6), dg::Point2ID(8)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(7), dg::Point2ID(5)), 1);
    VVS_CHECK_EQUL(map.getEdgeCost(dg::Point2ID(8), dg::Point2ID(7)), 1);

    VVS_CHECK_TRUE(map.getEdgeCost(dg::Point2ID(1), dg::Point2ID(3)) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(dg::Point2ID(2), dg::Point2ID(1)) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(dg::Point2ID(5), dg::Point2ID(6)) < 0);
    VVS_CHECK_TRUE(map.getEdgeCost(dg::Point2ID(5), dg::Point2ID(7)) < 0);

    return 0;
}

int testSimpleRoadPainter(bool verbose = false)
{
    // Build an example map
    dg::SimpleRoadMap map;
    map.addNode(dg::Point2ID(1, 0, 0)); // ID, x, y
    map.addNode(dg::Point2ID(2, 0, 1));
    map.addNode(dg::Point2ID(3, 1, 1));
    map.addNode(dg::Point2ID(4, 1, 0));
    map.addNode(dg::Point2ID(5, 2, 1));
    map.addNode(dg::Point2ID(6, 3, 1));
    map.addNode(dg::Point2ID(7, 2, 0));
    map.addNode(dg::Point2ID(8, 3, 0));
    map.addEdge(dg::Point2ID(1), dg::Point2ID(2));
    map.addEdge(dg::Point2ID(2), dg::Point2ID(3));
    map.addEdge(dg::Point2ID(3), dg::Point2ID(4));
    map.addEdge(dg::Point2ID(4), dg::Point2ID(1));
    map.addRoad(dg::Point2ID(3), dg::Point2ID(5)); // Add a bi-directional edge
    map.addEdge(dg::Point2ID(6), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(6), dg::Point2ID(8));
    map.addEdge(dg::Point2ID(7), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(8), dg::Point2ID(7));

    // Draw the map
    dg::MapPainter painter;
    cv::Mat image;
    VVS_CHECK_TRUE(painter.drawMap(image, map));
    VVS_CHECK_TRUE(image.empty() == false);

    // Draw additional nodes
    dg::CanvasInfo info = painter.getCanvasInfo(map);
    dg::SimpleRoadMap::NodeItr node1 = map.getHeadNode();
    VVS_CHECK_TRUE(node1 != map.getTailNode());
    VVS_CHECK_TRUE(painter.drawNode(image, info, node1->data, 0.1, 0.5, cx::COLOR_MAGENTA, 2));
    dg::SimpleRoadMap::Node* node3 = map.getNode(dg::Point2ID(3));
    VVS_CHECK_TRUE(node3 != NULL);
    VVS_CHECK_TRUE(painter.drawNode(image, info, node3->data, 0.1, 0.5, cx::COLOR_RED, -1));

    // Draw additional edges
    dg::SimpleRoadMap::EdgeItr edge = map.getHeadEdge(node1);
    VVS_CHECK_TRUE(edge != map.getTailEdge(node1));
    VVS_CHECK_TRUE(painter.drawEdge(image, info, node1->data, edge->to->data, 0.1, cx::COLOR_MAGENTA, 2, 0.1));

    // Draw a bigger image
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 500));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 5 * 0.5));
    cv::Mat bigger;
    VVS_CHECK_TRUE(painter.drawMap(bigger, map));
    VVS_CHECK_TRUE(bigger.empty() == false);

    if (verbose)
    {
        cv::imshow("Test SimpleRoadPainter", image);
        cv::waitKeyEx();
        cv::imshow("Test SimpleRoadPainter", bigger);
        cv::waitKeyEx();
    }
    return 0;
}

#endif // End of '__TEST_SIMPLE_ROAD_MAP__'
