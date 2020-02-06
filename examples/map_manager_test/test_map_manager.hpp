#ifndef __TEST_SIMPLE_MAP__
#define __TEST_SIMPLE_MAP__

#include "../unit_test/vvs.h"
#include "dg_map_manager.hpp"
#include <stdint.h>
#include <cstdint>

int testSimpleMapManager()
{
	dg::MapManager manager;
	//manager.load(128, 37, 19);
	//dg::NodeInfo node0(0, 128, 38);
	//node0.sv_ids = { 0, 1, 2 };
	//manager.getMap().addNode(node0);
	//manager.getMap().addNode(dg::NodeInfo(1, 1, 1));
	//dg::EdgeInfo edge01;
	//edge01.width = 0.5;
	//manager.getMap().addEdge(dg::NodeInfo(0), dg::NodeInfo(1), edge01);
	//manager.getMap().addEdge(dg::NodeInfo(1), dg::NodeInfo(0), edge01);

	//VVS_CHECK_EQUL(manager.getMap().countNodes(), 6);
	//VVS_CHECK_EQUL(manager.getMap().countEdges(dg::NodeInfo(0)), 1);

 //   // Rest the map
	//manager.getMap().removeAll();
 //   //VVS_CHECK_TRUE(manager.isEmpty());
 //   VVS_CHECK_EQUAL(manager.getMap().countNodes(), 0);

    // Load the map
	VVS_CHECK_TRUE(manager.load(36.384063, 127.374733, 650.0));
    //VVS_CHECK_TRUE(manager.load(36.383921, 127.367481, 16.0));
	VVS_CHECK_EQUL(manager.getMap().countNodes(), 4559); //8856); // 2);
	
	//manager.generatePath();
	dg::Path path = manager.getPath("test_simple_path.json");
	VVS_CHECK_EQUL(path.countPoints(), 78); // 9);

	dg::Map::Node* findNode = manager.getMap().findNode(559542564800735);
	VVS_CHECK_EQUL(findNode->data.id, 559542564800735);
	dg::Map::Edge* findEdge = manager.getMap().findEdge(559562564900779, 559562564900780);
	VVS_CHECK_EQUL(findEdge->cost.length, 13.370689140764002);
	
	//std::vector<cv::Point2d> poiloc = manager.getPOIloc("UST");
	
    return 0;
}

#endif // End of '__TEST_SIMPLE_MAP__'
