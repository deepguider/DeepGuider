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
#include "localizer/utm_converter.hpp"
#define M_PI 3.14159265358979323846

namespace dg
{

/**
 * @brief Simple map manager
 *
 * A <b>simple map manager</b> is defined with map information as dg::Map and path information with dg::Path.
 *
 * A map information contains its node, edge, POI and streetview information for the topological map.
 * A path information contains a sequence of points defined in PathElement for the path from origin to destination.
 */
class MapManager
{
public:
	/**
	 * The default constructor
	 */
	MapManager()
	{
		m_isMap = false;
		m_ip = "localhost";
		m_portErr = false;
	}

	/**
	 * The destructor
	 */
	~MapManager()
	{
		if (m_isMap)
		{
			delete m_map;
			m_isMap = false;
		}
	}

	/**
	 * Initialize to use this class
	 * @return True if successful (false if failed)
	 */
	bool initialize();

	/**
	 * Change the current server IP address
	 * @param ip The given server IP address to change
	 * @return True if successful (false if failed)
	 */
	bool setIP(std::string ip);

	/**
	 * Get the current server IP address
	 * @return The string of server IP address
	 */
	std::string getIP();

	/**
	 * Get the topological map within a certain radius based on latitude and longitude
	 * @param lat The given latitude of this topological map (Unit: [deg])
	 * @param lon The given longitude of this topological map (Unit: [deg])
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMap(double lat, double lon, double radius, Map& map);

	/**
	 * Get the topological map within a certain radius based on node
	 * @param node_id The given node ID of this topological map
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMap(ID node_id, double radius, Map& map);

	/**
	 * Get the topological map within a certain map tile
	 * @param tile The given map tile of this topological map
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMap(cv::Point2i tile, Map& map);

	/**
	 * Get the minimal topological map with a path
	 * @param path The given path of this topological map
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMap(Path path, Map& map);

	/**
	 * Get the current topological map
	 * @return A reference to gotten topological map
	 */
	Map& getMap();

	/**
	 * Get the path from the origin to the destination
	 * @param start_lat The given origin latitude of this path (Unit: [deg])
	 * @param start_lon The given origin longitude of this path (Unit: [deg])
	 * @param dest_lat The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon The given destination longitude of this path (Unit: [deg])
	 * @param path A reference to gotten path
	 * @param num_paths The number of paths requested (default: 2)
	 * @return True if successful (false if failed)
	 */
	bool getPath(double start_lat, double start_lon, double dest_lat, double dest_lon, Path& path, int num_paths = 2);
	
	/**
	 * Read the path from the given file
	 * @param filename The filename to read a path
	 * @param path A reference to gotten path
	 * @return True if successful (false if failed)
	 */
	bool getPath(const char* filename, Path& path);
	
	/**
	 * Get the current path
	 * @return A value to gotten path
	 */
	Path getPath();

	/**
	 * Get the POIs within a certain radius based on latitude and longitude
	 * @param lat The given latitude of these POIs (Unit: [deg])
	 * @param lon The given longitude of these POIs (Unit: [deg])
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @param poi_vec A reference to gotten POIs vector
	 * @return True if successful (false if failed)
	 */
	bool getPOI(double lat, double lon, double radius, std::vector<POI>& poi_vec);
	
	/**
	 * Get the POIs within a certain radius based on node
	 * @param node_id The given node ID of these POIs
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @param poi_vec A reference to gotten POIs vector
	 * @return True if successful (false if failed)
	 */
	bool getPOI(ID node_id, double radius, std::vector<POI>& poi_vec);
	
	/**
	 * Get the POIs within a certain map tile
	 * @param tile The given map tile of these POIs
	 * @param poi_vec A reference to gotten POIs vector
	 * @return True if successful (false if failed)
	 */
	bool getPOI(cv::Point2i tile, std::vector<POI>& poi_vec);
	
	/**
	 * Get the current POIs vector
	 * @return A reference to gotten POIs vector
	 */
	std::vector<POI>& getPOI();
	
	/**
	 * Get the POI corresponding to a certain POI ID
	 * @param poi_id The given POI ID of this POI
	 * @param poi A reference to gotten POI
	 * @return True if successful (false if failed)
	 */
	bool getPOI(ID poi_id, POI& poi);
	
	/**
	 * Get the POIs corresponding to a certain POI name
	 * @param poi_name The given POI name of this POI
	 * @param latlon The given latitude and longitude of this POI (Unit: [deg])
	 * @param radius The given radius of this POI (Unit: [m])
	 * @return A vector of gotten POIs
	 */
	std::vector<POI> getPOI(const std::string poi_name, LatLon latlon, double radius);

	/**
	 * Get the POIs corresponding to a certain POI name
	 * @param poi_name The given POI name of this POI
	 * @return A vector of gotten POIs
	 */
	std::vector<POI> getPOI(const std::string poi_name);

	//std::vector<cv::Point2d> getPOIloc(const char* poiname = "UST");

	/**
	 * Get the StreetViews within a certain radius based on latitude and longitude
	 * @param lat The given latitude of these StreetViews (Unit: [deg])
	 * @param lon The given longitude of these StreetViews (Unit: [deg])
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(double lat, double lon, double radius, std::vector<StreetView>& sv_vec);

	/**
	 * Get the StreetViews within a certain radius based on node
	 * @param node_id The given node ID of these StreetViews
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(ID node_id, double radius, std::vector<StreetView>& sv_vec);

	/**
	 * Get the StreetViews within a certain map tile
	 * @param tile The given map tile of these StreetViews
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(cv::Point2i tile, std::vector<StreetView>& sv_vec);

	/**
	 * Get the current StreetViews vector
	 * @return A reference to gotten StreetViews vector
	 */
	std::vector<StreetView>& getStreetView();

	/**
	 * Get the StreetView corresponding to a certain StreetView ID
	 * @param sv_id The given StreetView ID of this StreetView
	 * @param sv A reference to gotten StreetView
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(ID sv_id, StreetView& sv);

	/**
	 * Download the StreetView image corresponding to a certain StreetView ID
	 * @param sv_id The given StreetView ID of this StreetView image
	 * @param sv_image A reference to downloaded StreetView image
	 * @param cubic The face of an image cube - 360: "", front: "f", back: "b", left: "l", right: "r", up: "u", down: "d" (default: "")
	 * @param timeout The timeout value of curl (default: 10)
	 * @return True if successful (false if failed)
	 */
	bool getStreetViewImage(ID sv_id, cv::Mat& sv_image, std::string cubic = "", int timeout = 10);

protected:
	Map* m_map;
	Path m_path;
	std::string m_json;
	/** A hash table for finding LatLons */
	std::map<ID, LatLon> lookup_LatLons;
	/** A hash table for finding POIs */
	std::map<std::wstring, LatLon> lookup_pois;
	/** A hash table for finding StreetViews */
	std::map<ID, LatLon> lookup_svs;

	/*int lat2tiley(double lat, int z);
	int lon2tilex(double lon, int z);
	double tiley2lat(int y, int z);
	double tilex2lon(int x, int z);
	cv::Point2i latlon2xy(double lat, double lon, int z);*/


	/**
	 * Callback function for request to server 
	 * @param ptr A pointer to data
	 * @param size The size of a single data
	 * @param count The number of data
	 * @param stream A arbitrary user-data pointer
	 * @return The size of total data
	 */
	static size_t write_callback(void* ptr, size_t size, size_t count, void* stream);
		
	/**
	 * Request to server and receive response
	 * @param url A web address to request to the server
	 * @return True if successful (false if failed)
	 */
	bool query2server(std::string url);
	/*std::string to_utf8(uint32_t cp);
	bool decodeUni();*/
	
	/**
	 * Convert UTF8 to UTF16 format
	 * @param utf8 A pointer to data in UTF8 format
	 * @param utf16 A reference to data in UTF16 format
	 * @return True if successful (false if failed)
	 */
	bool utf8to16(const char* utf8, std::wstring& utf16);

	/**
	 * Request the topological map within a certain radius based on latitude and longitude to server and receive response 
	 * @param lat The given latitude of this topological map (Unit: [deg])
	 * @param lon The given longitude of this topological map (Unit: [deg])
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadMap(double lat, double lon, double radius);

	/**
	 * Request the topological map within a certain radius based on node to server and receive response
	 * @param node_id The given node ID of this topological map
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadMap(ID node_id, double radius);

	/**
	 * Request the topological map within a certain map tile to server and receive response
	 * @param tile The given map tile of this topological map
	 * @return True if successful (false if failed)
	 */
	bool downloadMap(cv::Point2i tile);

	/**
	 * Parse the topological map response received
	 * @param json A response received
	 * @return True if successful (false if failed)
	 */
	bool parseMap(const char* json);

	/**
	 * Request the path from the origin to the destination to server and receive response
	 * @param start_lat The given origin latitude of this path (Unit: [deg])
	 * @param start_lon The given origin longitude of this path (Unit: [deg])
	 * @param dest_lat The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon The given destination longitude of this path (Unit: [deg])
	 * @param num_paths The number of paths requested (default: 2)
	 * @return True if successful (false if failed)
	 */
	bool downloadPath(double start_lat, double start_lon, double dest_lat, double dest_lon, int num_paths = 2);

	/**
	 * Parse the path response received
	 * @param json A response received
	 * @return True if successful (false if failed)
	 */
	bool parsePath(const char* json);
	

	/**
	 * Receive the topological map including an incomplete path and completely rebuild the path
	 * @param start_lat The given origin latitude of this path (Unit: [deg])
	 * @param start_lon The given origin longitude of this path (Unit: [deg])
	 * @param dest_lat The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon The given destination longitude of this path (Unit: [deg])
	 * @param num_paths The number of paths requested (default: 2)
	 * @return True if successful (false if failed)
	 */
	bool generatePath(double start_lat, double start_lon, double dest_lat, double dest_lon, int num_paths = 2);

	/**
	 * Request the POIs within a certain radius based on latitude and longitude to server and receive response
	 * @param lat The given latitude of these POIs (Unit: [deg])
	 * @param lon The given longitude of these POIs (Unit: [deg])
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadPOI(double lat, double lon, double radius);

	/**
	 * Request the POIs within a certain radius based on node to server and receive response
	 * @param node_id The given node ID of these POIs
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadPOI(ID node_id, double radius);

	/**
	 * Request the POIs within a certain map tile to server and receive response
	 * @param tile The given map tile of these POIs
	 * @return True if successful (false if failed)
	 */
	bool downloadPOI(cv::Point2i tile);

	/**
	 * Parse the POIs response received
	 * @param json A response received
	 * @return True if successful (false if failed)
	 */
	bool parsePOI(const char* json);

	/**
	 * Request the StreetViews within a certain radius based on latitude and longitude to server and receive response
	 * @param lat The given latitude of these StreetViews (Unit: [deg])
	 * @param lon The given longitude of these StreetViews (Unit: [deg])
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadStreetView(double lat, double lon, double radius);

	/**
	 * Request the StreetViews within a certain radius based on node to server and receive response
	 * @param node_id The given node ID of these StreetViews
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadStreetView(ID node_id, double radius);

	/**
	 * Request the StreetViews within a certain map tile to server and receive response
	 * @param tile The given map tile of these StreetViews
	 * @return True if successful (false if failed)
	 */
	bool downloadStreetView(cv::Point2i tile);

	/**
	 * Parse the StreetViews response received
	 * @param json A response received
	 * @return True if successful (false if failed)
	 */
	bool parseStreetView(const char* json);

	/**
	 * Callback function for request to server
	 * @param ptr A pointer to data
	 * @param size The size of a single data
	 * @param nmemb The number of data
	 * @param userdata A arbitrary user-data pointer
	 * @return The size of total data
	 */
	static size_t writeImage_callback(char* ptr, size_t size, size_t nmemb, void* userdata);

	/**
	 * Request an image to server and download it
	 * @param url A web address to request to the server
	 * @param timeout The timeout value of curl (default: 10)
	 * @return The downloaded image
	 */
	cv::Mat queryImage2server(std::string url, int timeout = 10);

	/**
	 * Download the StreetView image corresponding to a certain StreetView ID
	 * @param sv_id The given StreetView ID of this StreetView image
	 * @param cubic The face of an image cube - 360: "", front: "f", back: "b", left: "l", right: "r", up: "u", down: "d" (default: "")
	 * @param timeout The timeout value of curl (default: 10)
	 * @param url_middle The web port number to request to the server (default: ":10000/")
	 * @return The downloaded image
	 */
	cv::Mat downloadStreetViewImage(ID sv_id, const std::string cubic = "", int timeout = 10, const std::string url_middle = ":10000/");

private:
	bool m_isMap;
	std::string m_ip;
	bool m_portErr;
};

class EdgeTemp : public Edge
{
public:
	//ID id;
	std::vector<ID> node_ids;
};

} // End of 'dg'

#endif // End of '__SIMPLE_MAP_MANAGER__'