#ifndef __SIMPLE_MAP_MANAGER__
#define __SIMPLE_MAP_MANAGER__

#include "dg_core.hpp"

// rapidjson header files
#include "rapidjson/document.h" 
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"
#include <fstream>
using namespace rapidjson;

#define CURL_STATICLIB
// curl header file
#include "curl/curl.h" 
#ifdef _WIN32
// curl library files
#ifdef _DEBUG
#pragma comment(lib, "libcurl_a_debug.lib")   // curl-7.65.3.zip 
#else
#pragma comment(lib, "libcurl_a.lib")         // curl-7.65.3.zip 
#endif
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Crypt32.lib")
#pragma comment(lib, "Wldap32.lib")
#pragma comment(lib, "Normaliz.lib")
#pragma execution_character_set( "utf-8" )
#include <atlstr.h> 
#endif

#define M_PI 3.14159265358979323846

namespace dg
{

class MapManager
{
public:
	/**
	 * The default constructor
	 */
	MapManager() {}

	int lat2tiley(double lat, int z);
	int lon2tilex(double lon, int z);
	double tiley2lat(int y, int z);
	double tilex2lon(int x, int z);
	cv::Point2i latlon2xy(double lat, double lon, int z);

	bool initialize();

	static size_t write_callback(void* ptr, size_t size, size_t count, void* stream);
	bool query2server(std::string url);
	bool downloadMap(double lat, double lon, double radius);
	bool downloadMap(ID node_id, double radius);
	bool downloadMap(cv::Point2i tile);
	std::string to_utf8(uint32_t cp);
	
	/**
	 * Read a map from the given file
	 * @param lon longitude
	 * @param lat latitude
	 * @param z zoom
	 * @return Result of success (true) or failure (false)
	 */
	bool load(double lat, double lon, double radius);
	///**
	// * Check whether this map is empty or not
	// * @return True if empty (true) or not (false)
	// */
	//bool isEmpty() const;

	bool decodeUni();
	bool downloadPath(double start_lat, double start_lon, double goal_lat, double goal_lon, int num_paths = 2);
	bool generatePath(double start_lat, double start_lon, double goal_lat, double goal_lon, int num_paths = 2);
	//Path getPath(const char* filename);
	Path getPath();
	Map& getMap(Path path);
	Map& getMap();
	bool downloadPOI(double lat, double lon, double radius);
	bool downloadPOI(ID node_id, double radius);
	bool downloadPOI(cv::Point2i tile);
	std::vector<cv::Point2d> getPOI(const char* poiname);
	//std::vector<cv::Point2d> getPOIloc(const char* poiname = "UST");

protected:
	Map m_map;
	Path m_path;

	std::string m_json;

	double m_lat = 0;
	double m_lon = 0;
	int m_z = 0;
};

class EdgeTemp : public Edge
{
public:
	//ID id;
	std::vector<ID> node_ids;
};

} // End of 'dg'

#endif // End of '__SIMPLE_MAP_MANAGER__'