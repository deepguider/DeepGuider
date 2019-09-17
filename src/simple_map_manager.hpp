#ifndef __SIMPLE_MAP_MANAGER__
#define __SIMPLE_MAP_MANAGER__

#include "basic_type.hpp"
#include "directed_graph.hpp"

// rapidjson header files
#include "rapidjson/document.h" 
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"
#include <fstream>
using namespace rapidjson;

#define M_PI 3.14159265358979323846

#include <iostream>
using namespace std;
#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
//#pragma comment(lib, "cpprest141_2_10")	// windows only
using namespace utility;              	// Common utilities like string conversions
using namespace web;                  	// Common features like URIs.
using namespace web::http;            	// Common HTTP functionality
using namespace web::http::client;    	// HTTP client features
using namespace concurrency::streams; 	// Asynchronous streams

//
//enum NodeType
//{
//	NODE_BASIC = 0,
//	NODE_CROSS = 1,
//	NODE_DOOR = 2,
//	NODE_ELEVATOR = 3
//};
//enum EdgeType
//{
//	EDGE_SIDEWALK = 0,
//	EDGE_STREET = 1,
//	EDGE_CROSSWALK = 2
//};

namespace dg
{

class NodeInfo
{
public:
	NodeInfo(uint64_t _id = 0, double _lon = -1, double _lat = -1, int _type = 0, int _floor = 0) : id(_id), lon(_lon), lat(_lat), type(_type), floor(_floor) { }

	bool operator==(const NodeInfo& rhs) const { return (id == rhs.id); }

	bool operator!=(const NodeInfo& rhs) const { return (id != rhs.id); }

	uint64_t id;
	double lon;
	double lat;
	int type;
	int floor;
	std::vector<uint64_t> sv_ids;
	std::vector<std::string> pois;
};

class EdgeInfo
{
public:
	EdgeInfo(uint64_t _id = 0, double _width = 3, double _length = 10, int _type = 0) : id(_id), width(_width), length(_length), type(_type) { }

	uint64_t id;
	double width;
	double length;
	int type;
};

class Map : public DirectedGraph<NodeInfo, EdgeInfo>
{
public:
    /**
     * The default constructor
     */
	Map();

	int long2tilex(double lon, int z);

	int lat2tiley(double lat, int z);

	double tilex2long(int x, int z);

	double tiley2lat(int y, int z);

	void downloadMap(cv::Point2i tile);

	cv::Point2i lonlat2xy(double lon, double lat, int z);

    /**
     * Read a map from the given file
     * @param lon longitude
	 * @param lat latitude
	 * @param z zoom
     * @return Result of success (true) or failure (false)
     */
    bool load(double lon = 128, double lat = 38, int z = 19);

    /**
     * Check whether this map is empty or not
     * @return True if empty (true) or not (false)
     */
    bool isEmpty() const;
};

class Path
{
public:
	/**
	 * The default constructor
	 */
	Path() {}

	/**
	 * Count the number of all points in the path (time complexity: O(1))
	 * @return The number of points
	 */
	int countPoints() const { return m_points.size(); }

	std::list<cv::Point2d> m_points;
};

class MapManager : public DirectedGraph<NodeInfo, EdgeInfo>
{
public:
	/**
	 * The default constructor
	 */
	MapManager() {}

	bool generatePath();
	Path getPath(const char* filename = "test_simple_Path.json");
	
	Map getMap(Path path);
	Map getMap(double lon = 128, double lat = 38, int z = 19);

	std::vector<cv::Point2d> getPOIloc(const char* poiname = "UST");

protected:
	Map m_map;
	Path m_path;

	double m_lon = 0;
	double m_lat = 0;
	int m_z = 0;
};

} // End of 'dg'

#endif // End of '__SIMPLE_MAP_MANAGER__'