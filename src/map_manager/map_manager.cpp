#include "map_manager.hpp"
#include <fstream>

using namespace rapidjson;

namespace dg
{

std::string MapManager::m_ip = "localhost";				// etri: 129.254.81.204
std::string MapManager::m_image_server_port = "10000";	// etri: 10000, coex: 10001, bucheon: 10002, etri_indoor: 10003
bool MapManager::m_portErr = false;


bool MapManager::initialize(const std::string ip, const std::string port)
{
	m_ip = ip;
	m_image_server_port = port;
	return true;
}

bool MapManager::setIP(const std::string ip)
{
	m_ip = ip;
	return true;
}

std::string MapManager::getIP()
{
	return m_ip;
}

bool MapManager::getTopoMap(double lat, double lon, double radius, Map& map)
{
	bool ok = downloadTopoMap(lat, lon, radius);
	if (!ok) return false;
	return parseTopoMap(m_json.c_str(), map);
}

bool MapManager::getTopoMap(ID node_id, double radius, Map& map)
{
	bool ok = downloadTopoMap(node_id, radius);
	if (!ok) return false;
	return parseTopoMap(m_json.c_str(), map);
}

bool MapManager::getTopoMap(cv::Point2i tile, Map& map)
{
	bool ok = downloadTopoMap(tile);
	if (!ok) return false;
	return parseTopoMap(m_json.c_str(), map);
}

bool MapManager::getTopoMap(const Path& path, Map& map, double margin)
{
	if (path.empty()) return false;

	// bounding rect of path points
	double min_x = path.pts[0].x;
	double max_x = path.pts[0].x;
	double min_y = path.pts[0].y;
	double max_y = path.pts[0].y;
	for (auto it = path.pts.begin(); it != path.pts.end(); it++)
	{
		if (it->x < min_x) min_x = it->x;
		if (it->x > max_x) max_x = it->x;
		if (it->y < min_y) min_y = it->y;
		if (it->y > max_y) max_y = it->y;
	}

	// check bounding rect of map points
	cv::Rect2d rc_map = map.getMapBoundingRect();
	if (rc_map.x <= min_x && rc_map.y <= min_y && (rc_map.x + rc_map.width) >= max_x && (rc_map.y + rc_map.height) >= max_y) return true;

	// get a new minimal map enclosing the path
	Point2 center_xy((min_x + max_x) / 2, (min_y + max_y) / 2);
	LatLon center_ll = toLatLon(center_xy);
	double radius = norm(center_xy - Point2(min_x, min_y)) + margin;
	return getTopoMap(center_ll.lat, center_ll.lon, radius, map);
}

bool MapManager::getTopoMap_expansion(const Path& path, Map& map, double margin)
{
	if (path.empty()) return false;

	// bounding rect of path points
	double min_x = path.pts[0].x;
	double max_x = path.pts[0].x;
	double min_y = path.pts[0].y;
	double max_y = path.pts[0].y;
	for (auto it = path.pts.begin(); it != path.pts.end(); it++)
	{
		if (it->x < min_x) min_x = it->x;
		if (it->x > max_x) max_x = it->x;
		if (it->y < min_y) min_y = it->y;
		if (it->y > max_y) max_y = it->y;
	}

	// check bounding rect of map points
	cv::Rect2d rc_map = map.getMapBoundingRect();
	if (rc_map.x <= min_x && rc_map.y <= min_y && (rc_map.x + rc_map.width) >= max_x && (rc_map.y + rc_map.height) >= max_y) return true;

	// bounding rect encolsing both map and path points
	cv::Rect2d rc_path_margined(min_x - margin, min_y - margin, max_x - min_x + 2 * margin, max_y - min_y + 2 * margin);
	cv::Rect2d rc_new = rc_map | rc_path_margined;

	// get a new expanded map
	Point2 center_xy(rc_new.x + rc_new.width / 2, rc_new.y + rc_new.height / 2);
	LatLon center_ll = toLatLon(center_xy);
	double radius = norm(center_xy - rc_new.tl());
	return getTopoMap(center_ll.lat, center_ll.lon, radius, map);
}

bool MapManager::downloadTopoMap(double lat, double lon, double radius)
{
	const std::string url_middle = ":21500/wgs/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadTopoMap(ID node_id, double radius)
{
	const std::string url_middle = ":21500/routing_node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadTopoMap(cv::Point2i tile)
{
	const std::string url_middle = ":21500/tile/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

class EdgeTemp : public Edge
{
public:
	std::vector<ID> node_ids;
};

bool MapManager::parseTopoMap(const char* json, Map& map)
{
	map.removeTopoMap();
	map.setReference(getReference());

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document.IsObject()) return false;
	const Value& features = document["features"];
	if (!features.IsArray()) return false;

	std::vector<EdgeTemp> temp_edge;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;
		std::string name = properties["name"].GetString();
		if (name == "edge") //continue; //TODO
		{
			EdgeTemp edge;
			edge.id = properties["id"].GetUint64();
			switch (properties["type"].GetInt())
			{
				/** Sidewalk */
			case 0: edge.type = Edge::EDGE_SIDEWALK; break;
				/** General road (e.g. roads shared by pedestrians and cars, street, alley, corridor, ...) */
			case 1: edge.type = Edge::EDGE_ROAD; break;
				/** Crosswalk */
			case 2: edge.type = Edge::EDGE_CROSSWALK; break;
				/** Elevator section */
			case 3: edge.type = Edge::EDGE_ELEVATOR; break;
				/** Escalator section */
			case 4: edge.type = Edge::EDGE_ESCALATOR; break;
				/** Stair section */
			case 5: edge.type = Edge::EDGE_STAIR; break;
			}
			edge.length = properties["length"].GetDouble();
			temp_edge.push_back(edge);
		}
	}

	int numNonEdges = 0;
	int numEdges = 0;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;
		std::string name = properties["name"].GetString();
		if (name == "Node")
		{
			Node node;
			node.id = properties["id"].GetUint64();
			switch (properties["type"].GetInt())
			{
				/** Basic node */
			case 0: node.type = Node::NODE_BASIC; break;
				/** Junction node (e.g. intersecting point, corner point, and end point of the road) */
			case 1: node.type = Node::NODE_JUNCTION; break;
				/** Door node (e.g. exit and entrance) */
			case 2: node.type = Node::NODE_DOOR; break;
				/** Elevator node */
			case 3: node.type = Node::NODE_ELEVATOR; break;
				/** Escalator node */
			case 4: node.type = Node::NODE_ESCALATOR; break;
			}
			node.floor = properties["floor"].GetInt();

			// swapped lat and lon
			LatLon ll;
			if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
			{
				ll.lon = properties["latitude"].GetDouble();
				ll.lat = properties["longitude"].GetDouble();
			}
			else
			{
				ll.lat = properties["latitude"].GetDouble();
				ll.lon = properties["longitude"].GetDouble();
			}
			node = toMetric(ll);

			const Value& edge_ids = properties["edge_ids"];
			for (Value::ConstValueIterator edge_id = edge_ids.Begin(); edge_id != edge_ids.End(); ++edge_id)
			{
				ID id = edge_id->GetUint64();
				std::vector<EdgeTemp>::iterator itr = std::find_if(temp_edge.begin(), temp_edge.end(), [id](EdgeTemp e) -> bool { return e.id == id; });
				if (itr != temp_edge.end())
					itr->node_ids.push_back(node.id);
			}
			map.addNode(node);
		}
	}

	for (std::vector<EdgeTemp>::iterator it = temp_edge.begin(); it < temp_edge.end(); it++)
	{
		if (it->node_ids.size() != 2) continue;
		map.addEdge(Edge(it->id, it->type, it->node_ids[0], it->node_ids[1], it->length, false));
	}

	return true;
}

bool MapManager::query2server(std::string url)
{
#ifdef _WIN32
	SetConsoleOutputCP(65001);
#endif

	curl_global_init(CURL_GLOBAL_ALL);
	CURL* curl = curl_easy_init();
	CURLcode res;

	m_json.clear();

	if (curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		std::string response;
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");
		curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5);

		// Perform the request, res will get the return code.
		res = curl_easy_perform(curl);

		// Always cleanup.
		curl_easy_cleanup(curl);
		curl_global_cleanup();

		// Check for errors.
		if (res != CURLE_OK)
		{
			fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
			return false;
		}
		else
		{
			m_json = response;
			return true;
		}
	}
	return false;
}

size_t MapManager::write_callback(void* ptr, size_t size, size_t count, void* stream)
{
	((std::string*)stream)->append((char*)ptr, 0, size * count);
	return size * count;
}

bool MapManager::getPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, int num_paths)
{
	path.clear();

	// by communication
	bool ok = downloadPath(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, num_paths);
	if (!ok) return false;
	if (invalid_response(m_json))
	{
		ok = downloadPath(round(start_lat * 1000) / 1000, round(start_lon * 1000) / 1000, start_floor, round(dest_lat * 1000) / 1000, round(dest_lon * 1000) / 1000, dest_floor, num_paths);
		if (invalid_response(m_json))
		{
			std::cout << "Invalid latitude or longitude!!" << std::endl;
			return false;
		}
		if (!ok) return false;
	}

	// convert to Path
	ok = parsePath(m_json.c_str(), path);
	if (!ok) return false;

	// append start point and dest point as PathNode
	Point2 xy1 = toMetric(LatLon(start_lat, start_lon));
	path.pts.insert(path.pts.begin(), PathNode(xy1, 0, 0));
	Point2 xy2 = toMetric(LatLon(dest_lat, dest_lon));
	path.pts.push_back(PathNode(xy2, 0, 0));

	return true;
}

bool MapManager::getPath_mapExpansion(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, Map& map, int num_paths)
{
	bool ok = getPath(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, path, num_paths);
	if (!ok) return false;

	ok = getTopoMap_expansion(path, map);
	if (!ok) return false;

	return true;
}

bool MapManager::getPath(const char* filename, Path& path)
{
	// Convert JSON document to string
	auto is = std::ifstream(filename, std::ifstream::in);
	assert(is.is_open());
	std::string line, text;
	while (std::getline(is, line))
	{
		text += line + "\n";
	}
	const char* json = text.c_str();
	is.close();

	// convert to Path
	return parsePath(json, path);
}

bool MapManager::downloadPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, int num_paths)
{
	const std::string url_middle = ":20005/"; // routing server (paths)
	std::string url = "http://" + m_ip + url_middle + std::to_string(start_lat) + "/" + std::to_string(start_lon) + "/" + std::to_string(start_floor) + "/" + std::to_string(dest_lat) + "/" + std::to_string(dest_lon) + "/" + std::to_string(dest_floor) + "/" + std::to_string(num_paths);

	return query2server(url);
}

bool MapManager::parsePath(const char* json, Path& path)
{
	path.clear();

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document[0].IsObject()) return false;
	const Value& features = document[0]["features"];
	if (!features.IsArray()) return false;

	Point2 xy;
	ID node_id = 0;
	ID edge_id = 0;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;
		std::string name = properties["name"].GetString();
		if (i % 2 == 0)	// node
		{
			if (!(name == "Node" || name == "node")) return false;
			node_id = properties["id"].GetUint64();

			// swapped lat and lon
			LatLon ll;
			if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
			{
				ll.lon = properties["latitude"].GetDouble();
				ll.lat = properties["longitude"].GetDouble();
			}
			else
			{
				ll.lat = properties["latitude"].GetDouble();
				ll.lon = properties["longitude"].GetDouble();
			}
			xy = toMetric(ll);

			if (!((i + 1) < features.Size()))
			{
				edge_id = 0;
				path.pts.push_back(PathNode(xy, node_id, edge_id));
			}
		}
		else			// edge
		{
			if (!(name == "Edge" || name == "edge")) return false;
			edge_id = properties["id"].GetUint64();
			path.pts.push_back(PathNode(xy, node_id, edge_id));
		}
	}

	return true;
}

bool MapManager::getPOI(double lat, double lon, double radius, Map& map)
{
	bool ok = downloadPOI(lat, lon, radius);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), map);
}

bool MapManager::getPOI(ID node_id, double radius, Map& map)
{
	bool ok = downloadPOI(node_id, radius);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), map);
}

bool MapManager::getPOI(cv::Point2i tile, Map& map)
{
	bool ok = downloadPOI(tile);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), map);
}

bool MapManager::getPOI(double lat, double lon, double radius, std::vector<POI>& poi_vec)
{
	bool ok = downloadPOI(lat, lon, radius);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), poi_vec);
}

bool MapManager::getPOI(ID node_id, double radius, std::vector<POI>& poi_vec)
{
	bool ok = downloadPOI(node_id, radius);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), poi_vec);
}

bool MapManager::getPOI(cv::Point2i tile, std::vector<POI>& poi_vec)
{
	bool ok = downloadPOI(tile);
	if (!ok) return false;
	return parsePOI(m_json.c_str(), poi_vec);
}

bool MapManager::downloadPOI(double lat, double lon, double radius)
{
	const std::string url_middle = ":21502/wgs/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(ID node_id, double radius)
{
	const std::string url_middle = ":21502/routing_node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(cv::Point2i tile)
{
	const std::string url_middle = ":21502/tile/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

bool utf8to16(const char* utf8, std::wstring& utf16)
{
	StringStream source(utf8);
	GenericStringBuffer<UTF16<> > target;

	bool hasError = false;
	while (source.Peek() != '\0')
		if (!Transcoder<UTF8<>, UTF16<> >::Transcode(source, target)) {
			hasError = true;
			break;
		}

	if (!hasError) {
		const wchar_t* t = target.GetString();
		utf16 = t;
	}

	return true;
}

bool MapManager::parsePOI(const char* json, Map& map)
{
	map.removePOIMap();
	map.setReference(getReference());

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document.IsObject()) return false;
	const Value& features = document["features"];
	if (!features.IsArray()) return false;

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;
		POI poi;
		poi.id = properties["id"].GetUint64();
		const char* utf8 = properties["name"].GetString();
		utf8to16(utf8, poi.name);
		poi.floor = properties["floor"].GetInt();

		// swapped lat and lon
		LatLon ll;
		if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
		{
			ll.lon = properties["latitude"].GetDouble();
			ll.lat = properties["longitude"].GetDouble();
		}
		else
		{
			ll.lat = properties["latitude"].GetDouble();
			ll.lon = properties["longitude"].GetDouble();
		}
		poi = toMetric(ll);

		map.addPOI(poi);
	}

	return true;
}

bool MapManager::parsePOI(const char* json, std::vector<POI>& poi_vec)
{
	poi_vec.clear();

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document.IsObject()) return false;
	const Value& features = document["features"];
	if (!features.IsArray()) return false;

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;

		POI poi;
		poi.id = properties["id"].GetUint64();
		const char* utf8 = properties["name"].GetString();
		utf8to16(utf8, poi.name);
		poi.floor = properties["floor"].GetInt();

		// swapped lat and lon
		LatLon ll;
		if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
		{
			ll.lon = properties["latitude"].GetDouble();
			ll.lat = properties["longitude"].GetDouble();
		}
		else
		{
			ll.lat = properties["latitude"].GetDouble();
			ll.lon = properties["longitude"].GetDouble();
		}
		poi = toMetric(ll);

		poi_vec.push_back(poi);
	}

	return true;
}

bool MapManager::getStreetView(double lat, double lon, double radius, Map& map)
{
	bool ok = downloadStreetView(lat, lon, radius);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), map);
}

bool MapManager::getStreetView(ID node_id, double radius, Map& map)
{
	bool ok = downloadStreetView(node_id, radius);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), map);
}

bool MapManager::getStreetView(cv::Point2i tile, Map& map)
{
	bool ok = downloadStreetView(tile);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), map);
}

bool MapManager::getStreetView(double lat, double lon, double radius, std::vector<StreetView>& sv_vec)
{
	bool ok = downloadStreetView(lat, lon, radius);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), sv_vec);
}

bool MapManager::getStreetView(ID node_id, double radius, std::vector<StreetView>& sv_vec)
{
	bool ok = downloadStreetView(node_id, radius);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), sv_vec);
}

bool MapManager::getStreetView(cv::Point2i tile, std::vector<StreetView>& sv_vec)
{
	bool ok = downloadStreetView(tile);
	if (!ok) return false;
	return parseStreetView(m_json.c_str(), sv_vec);
}

bool MapManager::downloadStreetView(double lat, double lon, double radius)
{
	const std::string url_middle = ":21501/wgs/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadStreetView(ID node_id, double radius)
{
	const std::string url_middle = ":21501/routing_node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadStreetView(cv::Point2i tile)
{
	const std::string url_middle = ":21501/tile/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

bool MapManager::parseStreetView(const char* json, Map& map)
{
	map.removeViewMap();
	map.setReference(getReference());

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document.IsObject()) return false;
	const Value& features = document["features"];
	if (!features.IsArray()) return false;

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;

		StreetView sv;
		std::string id_str = properties["id"].GetString();
		sv.id = std::strtoull(id_str.c_str(), nullptr, 0);
		std::string name = properties["name"].GetString();
		if (!(name == "streetview" || name == "StreetView")) return false;
		sv.floor = properties["floor"].GetInt();
		sv.date = properties["date"].GetString();
		sv.heading = properties["heading"].GetDouble();

		// swapped lat and lon
		LatLon ll;
		if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
		{
			ll.lon = properties["latitude"].GetDouble();
			ll.lat = properties["longitude"].GetDouble();
		}
		else
		{
			ll.lat = properties["latitude"].GetDouble();
			ll.lon = properties["longitude"].GetDouble();
		}
		sv = toMetric(ll);

		map.addView(sv);
	}

	return true;
}

bool MapManager::parseStreetView(const char* json, std::vector<StreetView>& sv_vec)
{
	sv_vec.clear();

	if (json == nullptr) return false;
	if (invalid_response(json)) return false;

	Document document;
	document.Parse(json);

	if (!document.IsObject()) return false;
	const Value& features = document["features"];
	if (!features.IsArray()) return false;

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if (!feature.IsObject()) return false;
		if (!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if (!properties.IsObject()) return false;

		StreetView sv;
		std::string id_str = properties["id"].GetString();
		sv.id = std::strtoull(id_str.c_str(), nullptr, 0);
		std::string name = properties["name"].GetString();
		if (!(name == "streetview" || name == "StreetView")) return false;
		sv.floor = properties["floor"].GetInt();
		sv.date = properties["date"].GetString();
		sv.heading = properties["heading"].GetDouble();

		// swapped lat and lon
		LatLon ll;
		if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
		{
			ll.lon = properties["latitude"].GetDouble();
			ll.lat = properties["longitude"].GetDouble();
		}
		else
		{
			ll.lat = properties["latitude"].GetDouble();
			ll.lon = properties["longitude"].GetDouble();
		}
		sv = toMetric(ll);

		sv_vec.push_back(sv);
	}

	return true;
}

bool MapManager::getStreetViewImage(ID sv_id, cv::Mat& sv_image, std::string cubic, int timeout)
{
	if (!(cubic == "f" || cubic == "b" || cubic == "l" || cubic == "r" || cubic == "u" || cubic == "d"))
		cubic = "";

	if ( m_image_server_port == "10003" ){  // 10003 for ETRI indoor
		if (cubic == "l") cubic = "0";  // More left
		if (cubic == "r") cubic = "2";  // Left
		if (cubic == "f") cubic = "1";  // Frontal
	}

	const std::string url_middle = ":" + m_image_server_port + "/";
	sv_image = downloadStreetViewImage(sv_id, cubic, timeout, url_middle);
	if (sv_image.empty())	return false;

	return true;
}

cv::Mat MapManager::downloadStreetViewImage(ID sv_id, const std::string cubic, int timeout, const std::string url_middle)
{
	if (cubic == "")
	{
		std::string url = "http://" + m_ip + url_middle + std::to_string(sv_id);
		return queryImage2server(url, timeout);
	}
	else
	{
		std::string url = "http://" + m_ip + url_middle + std::to_string(sv_id) + "/" + cubic;
		return queryImage2server(url, timeout);
	}
}

cv::Mat MapManager::queryImage2server(std::string url, int timeout)
{
#ifdef _WIN32
	SetConsoleOutputCP(65001);
#endif

	std::vector<uchar> stream;
	curl_global_init(CURL_GLOBAL_ALL);
	CURL* curl = curl_easy_init();
	CURLcode res;

	if (curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeImage_callback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream);
		curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);
		curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5);

		// Perform the request, res will get the return code.
		res = curl_easy_perform(curl);

		// Always cleanup.
		curl_easy_cleanup(curl);
		curl_global_cleanup();

		// Check for errors.
		if (res == CURLE_OK && !stream.empty())
		{
			const unsigned char* novalid = reinterpret_cast<const unsigned char*>("No valid");
			unsigned char part[8] = { stream[0], stream[1], stream[2], stream[3], stream[4], stream[5], stream[6], stream[7] };
			if (*part == *novalid)
				m_portErr = true;

			return cv::imdecode(stream, -1);
		}
	}

	return cv::Mat();
}

size_t MapManager::writeImage_callback(char* ptr, size_t size, size_t nmemb, void* userdata)
{
	std::vector<uchar>* stream = (std::vector<uchar>*)userdata;
	size_t count = size * nmemb;
	stream->insert(stream->end(), ptr, ptr + count);
	return count;
}


} // End of 'dg'

