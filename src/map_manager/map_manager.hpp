#ifndef __MAP_MANAGER__
#define __MAP_MANAGER__

#include "dg_core.hpp"
#include "rapidjson.hpp"
#include "curl.hpp"
#include "utils/utm_converter.hpp"

namespace dg
{

/**
 * @brief Map manager
 *
 * A <b>map manager</b> is defined with map information as dg::Map and path information with dg::Path.
 *
 * A map information contains its node, edge, POI and streetview information for the topological map.
 * A path information contains a sequence of points defined in PathNode for the path from start to destination.
 */
class MapManager : public UTMConverter
{
public:
	/**
	 * Initialize to use this class
	 * @param ip The given server IP address
	 * @return True if successful (false if failed)
	 */
	bool initialize(const std::string ip);

	/**
	 * Change the current server IP address
	 * @param ip The given server IP address to set
	 * @return True if successful (false if failed)
	 */
	bool setIP(std::string ip);

	/**
	 * Get the current server IP address
	 * @return The string of server IP address
	 */
	std::string getIP();

	/**
	 * Get a map within a radius from a given latitude and longitude point
	 * @param lat The given latitude (Unit: [deg])
	 * @param lon The given longitude (Unit: [deg])
	 * @param radius The given radius of map to get (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMapAll(double lat, double lon, double radius, Map& map)
	{
		bool ok = getTopoMap(lat, lon, radius, map);
		if (ok) ok = getPOI(lat, lon, radius, map);
		if (ok) ok = getStreetView(lat, lon, radius, map);
		return ok;
	}

	/**
	 * Get a map within a radius from a given node
	 * @param radius The given radius of map to get (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMapAll(ID node_id, double radius, Map& map)
	{
		bool ok = getTopoMap(node_id, radius, map);
		if (ok) ok = getPOI(node_id, radius, map);
		if (ok) ok = getStreetView(node_id, radius, map);
		return ok;
	}

	/**
	 * Get a map within a certain map tile
	 * @param tile The given map tile of map to get
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getMapAll(cv::Point2i tile, Map& map)
	{
		bool ok = getTopoMap(tile, map);
		if (ok) ok = getPOI(tile, map);
		if (ok) ok = getStreetView(tile, map);
		return ok;
	}

	/**
	 * Get a topological map within a radius from a given latitude and longitude point
	 * @param lat The given latitude (Unit: [deg])
	 * @param lon The given longitude (Unit: [deg])
	 * @param radius The given radius of map to get (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getTopoMap(double lat, double lon, double radius, Map& map);

	/**
	 * Get a topological map within a radius from a given node
	 * @param radius The given radius of map to get (Unit: [m])
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getTopoMap(ID node_id, double radius, Map& map);

	/**
	 * Get a topological map within a certain map tile
	 * @param tile The given map tile of map to get
	 * @param map A reference to gotten topological map
	 * @return True if successful (false if failed)
	 */
	bool getTopoMap(cv::Point2i tile, Map& map);

	/**
	 * Get a topological map that encloses a given path
	 * @param path A given path
	 * @param map A reference to gotten topological map
	 * @param margin A radius margin to gotten topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool getTopoMap(const Path& path, Map& map, double margin = 50.0);

	/**
	 * Expand a given map to enclose a given path if necessary (original map data is kept)
	 * @param path A given path
	 * @param map A reference to gotten topological map
	 * @param margin A radius margin to gotten topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool getTopoMap_expansion(const Path& path, Map& map, double margin = 50.0);

	/**
	 * Get a path from a given start and a destination point
	 * @param start_lat		The given origin latitude of this path (Unit: [deg])
	 * @param start_lon 	The given origin longitude of this path (Unit: [deg])
	 * @param start_floor	The given origin floor of this path  (Unit: [floor])
	 * @param dest_lat		The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon		The given destination longitude of this path (Unit: [deg])
	 * @param dest_floor	The given destination floor of this path  (Unit: [floor])
	 * @param path			A found path
	 * @param num_paths		The number of paths requested (default: 1)
	 * @return				True if successful (false if failed)
	 */
	bool getPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, int num_paths = 1);

	/**
	 * Get a path from a given start and a destination point and expand map accordingly to enclose the found path
	 * @param start_lat		The given origin latitude of this path (Unit: [deg])
	 * @param start_lon 	The given origin longitude of this path (Unit: [deg])
	 * @param start_floor	The given origin floor of this path  (Unit: [floor])
	 * @param dest_lat		The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon		The given destination longitude of this path (Unit: [deg])
	 * @param dest_floor	The given destination floor of this path  (Unit: [floor])
	 * @param path			A found path
	 * @param map			An expanded map to cover the found path
	 * @param num_paths		The number of paths requested (default: 1)
	 * @return				True if successful (false if failed)
	 */
	bool getPath_mapExpansion(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, Map& map, int num_paths = 1);

	/**
	 * Get a path from a saved json file
	 * @param filename		A given file name
	 * @param path			A found path
	 * @return				True if successful (false if failed)
	 */
	bool getPath(const char* filename, Path& path);

	/**
	 * Get the POIs within a certain radius based on latitude and longitude
	 * @param lat The given latitude of these POIs (Unit: [deg])
	 * @param lon The given longitude of these POIs (Unit: [deg])
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @param map A reference to gotten POIs map
	 * @return True if successful (false if failed)
	 */
	bool getPOI(double lat, double lon, double radius, Map& map);

	/**
	 * Get the POIs within a certain radius based on node
	 * @param node_id The given node ID of these POIs
	 * @param radius The given radius of these POIs (Unit: [m])
	 * @param map A reference to gotten POIs map
	 * @return True if successful (false if failed)
	 */
	bool getPOI(ID node_id, double radius, Map& map);

	/**
	 * Get the POIs within a certain map tile
	 * @param tile The given map tile of these POIs
	 * @param map A reference to gotten POIs map
	 * @return True if successful (false if failed)
	 */
	bool getPOI(cv::Point2i tile, Map& map);

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
	 * Get the StreetViews within a certain radius based on latitude and longitude
	 * @param lat The given latitude of these StreetViews (Unit: [deg])
	 * @param lon The given longitude of these StreetViews (Unit: [deg])
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(double lat, double lon, double radius, Map& map);

	/**
	 * Get the StreetViews within a certain radius based on node
	 * @param node_id The given node ID of these StreetViews
	 * @param radius The given radius of these StreetViews (Unit: [m])
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(ID node_id, double radius, Map& map);

	/**
	 * Get the StreetViews within a certain map tile
	 * @param tile The given map tile of these StreetViews
	 * @param sv_vec A reference to gotten StreetViews vector
	 * @return True if successful (false if failed)
	 */
	bool getStreetView(cv::Point2i tile, Map& map);

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
	 * Get a StreetView image corresponding to a certain StreetView ID
	 * @param sv_id The given StreetView ID of this StreetView image
	 * @param sv_image A reference to downloaded StreetView image
	 * @param cubic The face of an image cube - 360: "", front: "f", back: "b", left: "l", right: "r", up: "u", down: "d" (default: "")
	 * @param timeout The timeout value of curl (default: 10)
	 * @return True if successful (false if failed)
	 */
	static bool getStreetViewImage(ID sv_id, cv::Mat& sv_image, std::string cubic = "", int timeout = 10);

protected:
	static std::string m_ip;
	static bool m_portErr;
	std::string m_json;

	/**
	 * Request the topological map within a certain radius based on latitude and longitude to server and receive response
	 * @param lat The given latitude of this topological map (Unit: [deg])
	 * @param lon The given longitude of this topological map (Unit: [deg])
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadTopoMap(double lat, double lon, double radius);

	/**
	 * Request the topological map within a certain radius based on node to server and receive response
	 * @param node_id The given node ID of this topological map
	 * @param radius The given radius of this topological map (Unit: [m])
	 * @return True if successful (false if failed)
	 */
	bool downloadTopoMap(ID node_id, double radius);

	/**
	 * Request the topological map within a certain map tile to server and receive response
	 * @param tile The given map tile of this topological map
	 * @return True if successful (false if failed)
	 */
	bool downloadTopoMap(cv::Point2i tile);

	/**
	 * Parse the topological map response received
	 * @param json A response received
	 * @param map Parsed map from the json
	 * @return True if successful (false if failed)
	 */
	bool parseTopoMap(const char* json, Map& map);

	/**
	 * Request to server and receive response
	 * @param url A web address to request to the server
	 * @return True if successful (false if failed)
	 */
	bool query2server(std::string url);

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
	 * Request the path from the origin to the destination to server and receive response
	 * @param start_lat		The given origin latitude of this path (Unit: [deg])
	 * @param start_lon 	The given origin longitude of this path (Unit: [deg])
	 * @param start_floor	The given origin floor of this path  (Unit: [floor])
	 * @param dest_lat		The given destination latitude of this path (Unit: [deg])
	 * @param dest_lon		The given destination longitude of this path (Unit: [deg])
	 * @param dest_floor	The given destination floor of this path  (Unit: [floor])
	 * @param num_paths 	The number of paths requested (default: 2)
	 * @return 				True if successful (false if failed)
	 */
	bool downloadPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, int num_paths = 1);

	/**
	 * Parse the path response received
	 * @param json A response received
	 * @param path Parsed path from the json data
	 * @return True if successful (false if failed)
	 */
	bool parsePath(const char* json, Path& path);

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
	 * @param map Parsed map from the json
	 * @return True if successful (false if failed)
	 */
	bool parsePOI(const char* json, Map& map);

	/**
	 * Parse the POIs response received
	 * @param json A response received
	 * @param poi_vec Parsed POI's from the json
	 * @return True if successful (false if failed)
	 */
	bool parsePOI(const char* json, std::vector<POI>& poi_vec);

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
	 * @param map Parsed map from the json
	 * @return True if successful (false if failed)
	 */
	bool parseStreetView(const char* json, Map& map);

	/**
	 * Parse the StreetViews response received
	 * @param json A response received
	 * @param sv_vec Parsed StreetViews from the json
	 * @return True if successful (false if failed)
	 */
	bool parseStreetView(const char* json, std::vector<StreetView>& sv_vec);

	/**
	 * Download the StreetView image corresponding to a certain StreetView ID
	 * @param sv_id The given StreetView ID of this StreetView image
	 * @param cubic The face of an image cube - 360: "", front: "f", back: "b", left: "l", right: "r", up: "u", down: "d" (default: "")
	 * @param timeout The timeout value of curl (default: 10)
	 * @param url_middle The web port number to request to the server (default: ":10000/")
	 * @return The downloaded image
	 */
	static cv::Mat downloadStreetViewImage(ID sv_id, const std::string cubic = "", int timeout = 10, const std::string url_middle = ":10000/");

	/**
	 * Request an image to server and download it
	 * @param url A web address to request to the server
	 * @param timeout The timeout value of curl (default: 10)
	 * @return The downloaded image
	 */
	static cv::Mat queryImage2server(std::string url, int timeout = 10);

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
	 * Check if server response is valid or not
	 * @param json A given server response to check
	 * @return True of the response is valid
	 */
	bool invalid_response(const std::string& json)
	{
		if (json.empty() || json == "[]\n" || m_json == "{\"type\": \"FeatureCollection\", \"features\": []}\n") return true;
		return false;
	}
};

} // End of 'dg'

#endif // End of '__MAP_MANAGER__'
