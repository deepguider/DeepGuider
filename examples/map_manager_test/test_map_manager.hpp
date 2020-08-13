#ifndef __TEST_SIMPLE_MAP__
#define __TEST_SIMPLE_MAP__

#include "utils/vvs.h"
#include "dg_map_manager.hpp"
#include <stdint.h>
#include <cstdint>

int testSimpleMapManager()
{
	dg::MapManager manager;
	manager.initialize();
	bool ok;

	// Change the server IP address
	manager.setIP("129.254.87.96");
	std::string ip = manager.getIP();
	manager.setIP("localhost");
	ip = manager.getIP();
		
	// Get the path & map
	dg::Path path;
	ok = manager.getPath(36.381873, 127.36803, 36.384063, 127.374733, path);
	if (ok)
	{
		VVS_CHECK_EQUL(manager.getPath().pts.size(), 37);
		VVS_CHECK_EQUL(manager.getMap().nodes.size(), 236);
	}
	ok = manager.getPath_expansion(36.382423, 127.367433, 36.379444, 127.378857, path);
	if (ok)
	{
		VVS_CHECK_EQUL(manager.getPath().pts.size(), 65);
		VVS_CHECK_EQUL(manager.getMap().nodes.size(), 777);
	}

    // Get the map
	dg::Map map1, map2;
	ok = manager.getMap(path, map1);	
	if (ok)
		VVS_CHECK_EQUL(map1.nodes.size(), 777);
	ok = manager.getMap(36.384102, 127.374838, 700, map2);
	if (ok)
		VVS_CHECK_EQUL(map2.nodes.size(), 802);
	map1.set_union(map2);
	VVS_CHECK_EQUL(map1.nodes.size(), 983);
	std::vector<dg::Node> junc_node = manager.getMap_junction(dg::LatLon(36.384102, 127.374838), 3);

	// Get the path
	ok = manager.getPath("test_simple_path.json", path);
	if (ok)
		VVS_CHECK_EQUL(path.pts.size(), 37);
	
	// Find the node & edge
	dg::Node* findNode = manager.getMap().findNode(559562564900154);
	VVS_CHECK_EQUL(findNode->id, 559562564900154);
	dg::Edge* findEdge = manager.getMap().findEdge(559562564900154, 559562564900155);
	VVS_CHECK_EQUL(findEdge->length, 13.370689140764001);
	VVS_CHECK_EQUL((manager.getMap().findEdge(findEdge->id))->length, 13.370689140764001);

	// Get the POI
	std::vector<dg::POI> poi_vec;
	ok = manager.getPOI(36.384063, 127.374733, 650.0, poi_vec);
	if (ok)
		VVS_CHECK_EQUL(poi_vec.size(), 613);	// 36.382057170000003, 127.36764620000000, 10000.0, ip).size(), 8815);
	ok = manager.getPOI(559542565000236, 500.0, poi_vec);
	//if (ok)
	//	VVS_CHECK_EQUL(poi_vec.size(), 86);
	poi_vec = manager.getPOI(16099168);
	VVS_CHECK_EQUL(poi_vec[0].lat, 36.378127999999997);
	std::vector<dg::POI> pois = manager.getPOI("루이까스텔유성점");
	pois = manager.getPOI("우성이비에스", dg::LatLon(36.361303, 127.33648), 100.0);
	pois = manager.getPOI_sorting("루이까스텔유성점", dg::LatLon(36.384063, 127.374733));
	pois = manager.getPOI_sorting("우성이비에스", dg::LatLon(36.361303, 127.33648), 100.0, dg::LatLon(36.384063, 127.374733));
	//std::vector<cv::Point2d> poiloc = manager.getPOIloc("UST");
	
	// Get the StreetView
	std::vector<dg::StreetView> sv_vec;
	ok = manager.getStreetView(36.384063, 127.374733, 650.0, sv_vec);
	if (ok)
		VVS_CHECK_EQUL(sv_vec.size(), 2144);	// 36.382057170000003, 127.36764620000000, 10000.0, ip).size(), 36607);
	ok = manager.getStreetView(559542565000236, 500.0, sv_vec);
	if (ok)
		VVS_CHECK_EQUL(sv_vec.size(), 630);
	sv_vec = manager.getStreetView(32364501511);
	VVS_CHECK_EQUL(sv_vec[0].heading, 270.360000);

	// Get the StreetView image
	cv::Mat sv_image;
	ok = manager.getStreetViewImage(14255003037, sv_image, "");	// f, b, l, r, u, d

    return 0;
}

#endif // End of '__TEST_SIMPLE_MAP__'
