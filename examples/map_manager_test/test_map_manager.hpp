#ifndef __TEST_SIMPLE_MAP__
#define __TEST_SIMPLE_MAP__

#include "vvs.h"
#include "simple_mappath.hpp"

int testSimpleMapManager(/*const char* filename = "test_simple_map.json"*/)
{
	dg::Map map;
	dg::NodeInfo node0(0, 128, 38);
	node0.sv_ids = { 0, 1, 2 };
	map.addNode(node0);
	map.addNode(dg::NodeInfo(1, 1, 1));
	dg::EdgeInfo edge01;
	edge01.width = 0.5;
	map.addEdge(dg::NodeInfo(0), dg::NodeInfo(1), edge01);
	map.addEdge(dg::NodeInfo(1), dg::NodeInfo(0), edge01);

	VVS_CHECK_EQUL(map.countNodes(), 2);
	VVS_CHECK_EQUL(map.countEdges(dg::NodeInfo(0)), 1);

    // Rest the map
    map.removeAll();
    VVS_CHECK_TRUE(map.isEmpty());
    VVS_CHECK_EQUAL(map.countNodes(), 0);

    // Load the map
    VVS_CHECK_TRUE(map.load(128, 38));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_EQUL(map.countNodes(), 4);
	
	dg::MapManager manager;
	dg::Path path =	manager.getPath("test_simple_Path.json");
	VVS_CHECK_EQUL(path.countPoints(), 9);

	std::vector<cv::Point2d> poiloc = manager.getPOIloc("UST");

	manager.generatePath();

    return 0;
}

#endif // End of '__TEST_SIMPLE_MAP__'
