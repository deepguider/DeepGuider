#ifndef __TEST_SIMPLE_MAP__
#define __TEST_SIMPLE_MAP__

#include "utils/vvs.h"
#include "dg_map_manager.hpp"
#include <stdint.h>
#include <cstdint>

int testSimpleMapManager()
{
	dg::MapManager manager;
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
		VVS_CHECK_EQUL(manager.getMap().nodes.size(), 201);
	}

    // Get the map
	dg::Map map;
	ok = manager.getMap(559542565000236, 700, map);
	if (ok)
		VVS_CHECK_EQUL(map.nodes.size(), 747);
	ok = manager.getMap(36.382967999999998, 127.37138150000001, 700, map);
	if (ok)
		VVS_CHECK_EQUL(map.nodes.size(), 799);// 36.384063, 127.374733, 650.0));//

	// Get the path
	ok = manager.getPath("test_simple_path.json", path);
	if (ok)
		VVS_CHECK_EQUL(path.pts.size(), 37);
	
	// Find the node & edge
	dg::Node* findNode = manager.getMap().findNode(559542565000236);
	VVS_CHECK_EQUL(findNode->id, 559542565000236);
	dg::Edge* findEdge = manager.getMap().findEdge(559542565000236, 559542565000238);// 559562564900154, 559562564900155);
	VVS_CHECK_EQUL(findEdge->length, 31.267266147580031);// 13.370689140764002);
	VVS_CHECK_EQUL((manager.getMap().findEdge(findEdge->id))->length, 31.267266147580031);// 13.370689140764002);

	// Get the POI
	std::list<dg::POI> poi_list;
	ok = manager.getPOI(36.384063, 127.374733, 650.0, poi_list);
	if (ok)
		VVS_CHECK_EQUL(poi_list.size(), 613);	// 36.382057170000003, 127.36764620000000, 10000.0, ip).size(), 8815);
	ok = manager.getPOI(559542565000236, 500.0, poi_list);
	if (ok)
		VVS_CHECK_EQUL(poi_list.size(), 86);
	dg::POI poi;
	ok = manager.getPOI(16099168, poi);
	if (ok)
		VVS_CHECK_EQUL(poi.lat, 36.378127999999997);
	//std::vector<cv::Point2d> poiloc = manager.getPOIloc("UST");
	
	// Get the StreetView
	std::list<dg::StreetView> sv_list;
	ok = manager.getStreetView(36.384063, 127.374733, 650.0, sv_list);
	if (ok)
		VVS_CHECK_EQUL(sv_list.size(), 2144);	// 36.382057170000003, 127.36764620000000, 10000.0, ip).size(), 36607);
	ok = manager.getStreetView(559542565000236, 500.0, sv_list);
	if (ok)
		VVS_CHECK_EQUL(sv_list.size(), 630);
	dg::StreetView sv;
	ok = manager.getStreetView(32364501511, sv);
	if (ok)
		VVS_CHECK_EQUL(sv.heading, 270.360000);

	// Get the StreetView image
	cv::Mat sv_image;
	ok = manager.getStreetViewImage(14255003037, sv_image, "");	// f, b, l, r, u, d

    return 0;
}

#endif // End of '__TEST_SIMPLE_MAP__'
