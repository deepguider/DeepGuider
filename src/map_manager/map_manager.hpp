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

	~MapManager()
	{
		if (m_isMap)
		{
			delete m_map;
			m_isMap = false;
		}
	}

	bool initialize();

	void setIP(std::string ip);
	std::string getIP();

	bool getMap(double lat, double lon, double radius, Map& map);
	bool getMap(Path path, Map& map);
	Map& getMap();

	bool getPath(double start_lat, double start_lon, double dest_lat, double dest_lon, Path& path, int num_paths = 2);
	bool getPath(const char* filename, Path& path);
	Path getPath();

	bool getPOI(double lat, double lon, double radius, std::list<POI>& poi_list);
	bool getPOI(ID node_id, double radius, std::list<POI>& poi_list);
	std::list<POI>& getPOI();
	bool getPOI(ID poi_id, POI& poi);
	//std::vector<cv::Point2d> getPOIloc(const char* poiname = "UST");

	bool getStreetView(double lat, double lon, double radius, std::list<StreetView>& sv_list);
	bool getStreetView(ID node_id, double radius, std::list<StreetView>& sv_list);
	std::list<StreetView>& getStreetView();
	bool getStreetView(ID sv_id, StreetView& sv);
	bool getStreetViewImage(ID sv_id, cv::Mat& sv_image, std::string cubic = "", int timeout = 10);

protected:
	Map* m_map;
	Path m_path;
	std::string m_json;

	/*int lat2tiley(double lat, int z);
	int lon2tilex(double lon, int z);
	double tiley2lat(int y, int z);
	double tilex2lon(int x, int z);
	cv::Point2i latlon2xy(double lat, double lon, int z);*/

	static size_t write_callback(void* ptr, size_t size, size_t count, void* stream);
	bool query2server(std::string url);
	/*std::string to_utf8(uint32_t cp);
	bool decodeUni();*/
	bool utf8to16(const char* utf8, std::wstring& utf16);
	bool downloadMap(double lat, double lon, double radius);
	bool downloadMap(ID node_id, double radius);
	bool downloadMap(cv::Point2i tile);
	bool parseMap(const char* json);
	/**
	 * Read a map from the given file
	 * @param lon longitude
	 * @param lat latitude
	 * @param z zoom
	 * @return Result of success (true) or failure (false)
	 */
	bool loadMap(double lat, double lon, double radius);

	bool downloadPath(double start_lat, double start_lon, double dest_lat, double dest_lon, int num_paths = 2);
	bool parsePath(const char* json);
	bool generatePath(double start_lat, double start_lon, double dest_lat, double dest_lon, int num_paths = 2);

	bool downloadPOI(double lat, double lon, double radius);
	bool downloadPOI(ID node_id, double radius);
	bool downloadPOI(cv::Point2i tile);
	bool parsePOI(const char* json);

	bool downloadStreetView(double lat, double lon, double radius);
	bool downloadStreetView(ID node_id, double radius);
	bool downloadStreetView(cv::Point2i tile);
	bool parseStreetView(const char* json);

	static size_t writeImage_callback(char* ptr, size_t size, size_t nmemb, void* userdata);
	cv::Mat queryImage2server(std::string url, int timeout = 10);
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