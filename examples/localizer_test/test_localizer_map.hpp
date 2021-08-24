#ifndef __TEST_LOCALIZER_ROAD__
#define __TEST_LOCALIZER_ROAD__

#include "dg_localizer.hpp"
#include "utils/map_painter.hpp"
#include "utils/vvs.h"

int testLocMap(const char* filename = "test_simple_road_map.csv")
{
    // An example road map ('+' represents direction of edges)
    // 2 --+ 3 +-+ 5 +-- 6
    // +     |     +     |
    // |     +     |     +
    // 1 +-- 4     7 +-- 8

    // Test degenerate cases
    dg::Map map;
    VVS_CHECK_TRUE(map.load("nothing") == false);
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_TRUE(map.save(filename) == false);

    // Build and save a map
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0))); // ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)));

    dg::Node* node1_ptr = map.getNode(1);
    dg::Node* node2_ptr = map.getNode(2);
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(9, 0, 1, 2))); // Given: edge ID, type, node1, node2, length
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(10, 0, 1, 4)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(11, 0, 2, 3)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(12, 0, 3, 4)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(13, 0, 3, 5)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(14, 0, 5, 6)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(15, 0, 5, 7)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(16, 0, 6, 8)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(17, 0, 7, 8)));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);
    VVS_CHECK_TRUE(map.save(filename, false));

    // Copy the map
    dg::Map copy = map;
    VVS_CHECK_EQUL(copy.countNodes(), map.countNodes());

    // Rest the map
    map.removeAll();
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_EQUAL(map.countNodes(), 0);

    // Load the map
    VVS_CHECK_TRUE(map.load(filename, false));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 8);

    // Check each node data
    VVS_CHECK_EQUL(map.getNode(1)->x, 0);
    VVS_CHECK_EQUL(map.getNode(1)->y, 0);
    VVS_CHECK_EQUL(map.getNode(2)->x, 0);
    VVS_CHECK_EQUL(map.getNode(2)->y, 1);
    VVS_CHECK_EQUL(map.getNode(3)->x, 1);
    VVS_CHECK_EQUL(map.getNode(3)->y, 1);
    VVS_CHECK_EQUL(map.getNode(4)->x, 1);
    VVS_CHECK_EQUL(map.getNode(4)->y, 0);
    VVS_CHECK_EQUL(map.getNode(5)->x, 2);
    VVS_CHECK_EQUL(map.getNode(5)->y, 1);
    VVS_CHECK_EQUL(map.getNode(6)->x, 3);
    VVS_CHECK_EQUL(map.getNode(6)->y, 1);
    VVS_CHECK_EQUL(map.getNode(7)->x, 2);
    VVS_CHECK_EQUL(map.getNode(7)->y, 0);
    VVS_CHECK_EQUL(map.getNode(8)->x, 3);
    VVS_CHECK_EQUL(map.getNode(8)->y, 0);

    // Check each connectivity and cost
    VVS_CHECK_EQUL(map.getEdge(1, 2)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(2, 3)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(3, 4)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(4, 1)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(3, 5)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(5, 3)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(6, 5)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(6, 8)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(7, 5)->length, 1);
    VVS_CHECK_EQUL(map.getEdge(8, 7)->length, 1);

    VVS_CHECK_TRUE(map.getEdge(1, 3) == nullptr);
    VVS_CHECK_TRUE(map.getEdge(2, 1) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(5, 6) != nullptr);
    VVS_CHECK_TRUE(map.getEdge(5, 7) != nullptr);

    return 0;
}

int testLocMapPainter(int wait_msec = 1)
{
    // Build an example map
    dg::Map map;
    VVS_CHECK_TRUE(map.addNode(dg::Node(1, 0, 0))); // ID, x, y
    VVS_CHECK_TRUE(map.addNode(dg::Node(2, 0, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(3, 1, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(4, 1, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(5, 2, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(6, 3, 1)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(7, 2, 0)));
    VVS_CHECK_TRUE(map.addNode(dg::Node(8, 3, 0)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(9, 0, 1, 2))); // Given: edge ID, type, node1, node2, length
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(10, 0, 1, 4)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(11, 0, 2, 3)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(12, 0, 3, 4)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(13, 0, 3, 5)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(14, 0, 5, 6)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(15, 0, 5, 7)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(16, 0, 6, 8)));
    VVS_CHECK_TRUE(map.addEdge(dg::Edge(17, 0, 7, 8)));
    VVS_CHECK_TRUE(map.isEmpty() == false);

    // Draw the map
    dg::MapPainter painter;
    cv::Mat image = cv::Mat::zeros(250, 500, CV_8UC3);
    image = 255;
    VVS_CHECK_TRUE(painter.configCanvas(dg::Point2(100, 180), cv::Point2d(100, 100), image.size(), 0, 0));
    VVS_CHECK_TRUE(painter.drawMap(image, &map));
    VVS_CHECK_TRUE(image.empty() == false);

    // Draw additional nodes
    auto node1 = map.getHeadNode();
    VVS_CHECK_TRUE(node1 != map.getTailNode());
    VVS_CHECK_TRUE(painter.drawNode(image, *node1, 3, 0.5, cx::COLOR_MAGENTA));
    dg::Node* node3 = map.getNode(3);
    VVS_CHECK_TRUE(node3 != nullptr);
    VVS_CHECK_TRUE(painter.drawNode(image, *node3, 3, 0.5, cx::COLOR_RED));

    // Draw additional edges
    auto edge = map.getEdge(&(*node1), 0);
    VVS_CHECK_TRUE(edge != map.getEdge(node1->edge_ids.back()));
    dg::Node* to = map.getConnectedNode(&(*node1), edge->id);
    VVS_CHECK_TRUE(painter.drawEdge(image, *node1, *to, 3, cx::COLOR_MAGENTA, 2));

    // Draw a bigger image

    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 5 * 0.5));
    cv::Mat bigger = cv::Mat::zeros(500, 1000, CV_8UC3);
    bigger = 255;
    VVS_CHECK_TRUE(painter.configCanvas(dg::Point2(200, 360), cv::Point2d(200, 200), bigger.size(), 0, 0));
    VVS_CHECK_TRUE(painter.drawMap(bigger, &map));
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
