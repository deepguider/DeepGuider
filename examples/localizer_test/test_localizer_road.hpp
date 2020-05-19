#ifndef __TEST_LOCALIZER_ROAD__
#define __TEST_LOCALIZER_ROAD__

#include "vvs.h"
#include "dg_localizer.hpp"

int testLocRoadMap(const char* filename = "test_simple_road_map.csv")
{
    // An example road map ('+' represents direction of edges)
    // 2 --+ 3 +-+ 5 +-- 6
    // +     |     +     |
    // |     +     |     +
    // 1 +-- 4     7 +-- 8

    // Test degenerate cases
    dg::RoadMap map;
    VVS_CHECK_TRUE(map.load("nothing") == false);
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_TRUE(map.save(filename) == false);

    // Build and save a map
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(1, 0, 0)) != NULL); // ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(2, 0, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(3, 1, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(4, 1, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(5, 2, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(6, 3, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(7, 2, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(8, 3, 0)) != NULL);

    dg::RoadMap::Node* node1_ptr = map.getNode(dg::Point2ID(1));
    dg::RoadMap::Node* node2_ptr = map.getNode(2);
    VVS_CHECK_TRUE(map.addEdge(node1_ptr, node2_ptr) != NULL);              // Method #1 to add an edge (pointer)
    VVS_CHECK_TRUE(map.addEdge(dg::Point2ID(2), dg::Point2ID(3)) != NULL);  // Method #2 to add an edge (Point2ID)
    VVS_CHECK_TRUE(map.addEdge(3, 4) != NULL);                              // Method #3 to add an edge (ID)
    VVS_CHECK_TRUE(map.addEdge(4, 1) != NULL);
    VVS_CHECK_TRUE(map.addRoad(3, 5));                                      // Add a bi-directional edge
    VVS_CHECK_TRUE(map.addEdge(6, 5) != NULL);
    VVS_CHECK_TRUE(map.addEdge(6, 8) != NULL);
    VVS_CHECK_TRUE(map.addEdge(7, 5) != NULL);
    VVS_CHECK_TRUE(map.addEdge(8, 7) != NULL);
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);
    VVS_CHECK_TRUE(map.save(filename));

    // Copy the map
    dg::RoadMap copy = map;
    VVS_CHECK_EQUL(copy.countNodes(), map.countNodes());

    // Rest the map
    map.removeAll();
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_EQUAL(map.countNodes(), 0);

    // Load the map
    VVS_CHECK_TRUE(map.load(filename));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);

    // Check each node data
    VVS_CHECK_EQUL(map.getNode(1)->data.x, 0);
    VVS_CHECK_EQUL(map.getNode(1)->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(2)->data.x, 0);
    VVS_CHECK_EQUL(map.getNode(2)->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(3)->data.x, 1);
    VVS_CHECK_EQUL(map.getNode(3)->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(4)->data.x, 1);
    VVS_CHECK_EQUL(map.getNode(4)->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(5)->data.x, 2);
    VVS_CHECK_EQUL(map.getNode(5)->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(6)->data.x, 3);
    VVS_CHECK_EQUL(map.getNode(6)->data.y, 1);
    VVS_CHECK_EQUL(map.getNode(7)->data.x, 2);
    VVS_CHECK_EQUL(map.getNode(7)->data.y, 0);
    VVS_CHECK_EQUL(map.getNode(8)->data.x, 3);
    VVS_CHECK_EQUL(map.getNode(8)->data.y, 0);

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

int testLocRoadPainter(int wait_msec = 1)
{
    // Build an example map
    dg::RoadMap map;
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(1, 0, 0)) != NULL); // ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(2, 0, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(3, 1, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(4, 1, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(5, 2, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(6, 3, 1)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(7, 2, 0)) != NULL);
    VVS_CHECK_TRUE(map.addNode(dg::Point2ID(8, 3, 0)) != NULL);
    VVS_CHECK_TRUE(map.addEdge(1, 2) != NULL);
    VVS_CHECK_TRUE(map.addEdge(2, 3) != NULL);
    VVS_CHECK_TRUE(map.addEdge(3, 4) != NULL);
    VVS_CHECK_TRUE(map.addEdge(4, 1) != NULL);
    VVS_CHECK_TRUE(map.addRoad(3, 5)); // Add a bi-directional edge
    VVS_CHECK_TRUE(map.addEdge(6, 5) != NULL);
    VVS_CHECK_TRUE(map.addEdge(6, 8) != NULL);
    VVS_CHECK_TRUE(map.addEdge(7, 5) != NULL);
    VVS_CHECK_TRUE(map.addEdge(8, 7) != NULL);
    VVS_CHECK_TRUE(map.isEmpty() == false);

    // Draw the map
    dg::SimpleRoadPainter painter;
    cv::Mat image;
    VVS_CHECK_TRUE(painter.drawMap(image, map));
    VVS_CHECK_TRUE(image.empty() == false);

    // Draw additional nodes
    dg::CanvasInfo info = painter.getCanvasInfo(map, image.size());
    dg::RoadMap::NodeItr node1 = map.getHeadNode();
    VVS_CHECK_TRUE(node1 != map.getTailNode());
    VVS_CHECK_TRUE(painter.drawNode(image, info, node1->data, 0.1, 0.5, cx::COLOR_MAGENTA, 2));
    dg::RoadMap::Node* node3 = map.getNode(dg::Point2ID(3));
    VVS_CHECK_TRUE(node3 != NULL);
    VVS_CHECK_TRUE(painter.drawNode(image, info, node3->data, 0.1, 0.5, cx::COLOR_RED, -1));

    // Draw additional edges
    dg::RoadMap::EdgeItr edge = map.getHeadEdge(node1);
    VVS_CHECK_TRUE(edge != map.getTailEdge(node1));
    VVS_CHECK_TRUE(painter.drawEdge(image, info, node1->data, edge->to->data, 0.1, cx::COLOR_MAGENTA, 2, 0.1));

    // Draw a bigger image
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 500));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 5 * 0.5));
    cv::Mat bigger;
    VVS_CHECK_TRUE(painter.drawMap(bigger, map));
    VVS_CHECK_TRUE(bigger.empty() == false);

    if (wait_msec >= 0)
    {
        cv::imshow("testLocRoadPainter", image);
        cv::waitKey(wait_msec);
        cv::imshow("testLocRoadPainter", bigger);
        cv::waitKey(wait_msec);
    }
    return 0;
}

#endif // End of '__TEST_LOCALIZER_ROAD__'
