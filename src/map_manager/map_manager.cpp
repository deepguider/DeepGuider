#include "map_manager.hpp"

namespace dg
{


bool MapManager::initialize()
{
	m_map = new Map();
	m_isMap = true;

	std::vector<POI> poi_vec;
	bool ok = getPOI(36.384063, 127.374733, 40000.0, poi_vec);	// Korea
	if (!ok)
	{
		delete m_map;
		m_isMap = false;

		return false;
	}	
	for (std::vector<POI>::iterator it = m_map->pois.begin(); it != m_map->pois.end(); ++it)
	{
		lookup_pois_name.insert(std::make_pair(it->name, LatLon(it->lat, it->lon)));
		//lookup_pois_id.insert(std::make_pair(it->id, LatLon(it->lat, it->lon)));
	}
	m_map->pois.clear();

	//std::vector<StreetView> sv_vec;
	//ok = getStreetView(36.384063, 127.374733, 40000.0, sv_vec);	// Korea
	//if (!ok)
	//{
	//	delete m_map;
	//	m_isMap = false;

	//	return false;
	//}
	//for (std::vector<StreetView>::iterator it = m_map->views.begin(); it != m_map->views.end(); ++it)
	//{
	//	lookup_svs.insert(std::make_pair(it->id, LatLon(it->lat, it->lon)));
	//}
	//m_map->views.clear();

	return true;
}

bool MapManager::setIP(const std::string ip)
{
	m_ip = ip;

	if (m_ip == ip)
		return true;
	else
		return false;
}

std::string MapManager::getIP()
{
	return m_ip;
}

//int MapManager::lat2tiley(double lat, int z)
//{
//	double latrad = lat * M_PI / 180.0;
//	return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)));
//}
//
//int MapManager::lon2tilex(double lon, int z)
//{
//	return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
//}
//
//double MapManager::tiley2lat(int y, int z)
//{
//	double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
//	return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
//}
//
//double MapManager::tilex2lon(int x, int z)
//{
//	return x / (double)(1 << z) * 360.0 - 180;
//}
//
//cv::Point2i MapManager::latlon2xy(double lat, double lon, int z)
//{
//	return cv::Point2i(lon2tilex(lon, z), lat2tiley(lat, z));
//}

size_t MapManager::write_callback(void* ptr, size_t size, size_t count, void* stream)
{
	((std::string*)stream)->append((char*)ptr, 0, size * count);
	return size * count;
}
	
bool MapManager::query2server(std::string url)
{
#ifdef _WIN32
	SetConsoleOutputCP(65001);
#endif
		
	curl_global_init(CURL_GLOBAL_ALL);
	CURL* curl = curl_easy_init();
	CURLcode res;

	if (curl)
	{
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		std::string response;
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

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
			//fprintf(stdout, "%s\n", response.c_str());
			m_json = response;
		}
	}

	return true;
}

//// unicode-escape decoding
//std::string MapManager::to_utf8(uint32_t cp)
//{
//	/*
//	if using C++11 or later, you can do this:
//
//	std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
//	return conv.to_bytes( (char32_t)cp );
//
//	Otherwise...
//	*/
//
//	std::string result;
//
//	int count;
//	if (cp < 0x0080)
//		count = 1;
//	else if (cp < 0x0800)
//		count = 2;
//	else if (cp < 0x10000)
//		count = 3;
//	else if (cp <= 0x10FFFF)
//		count = 4;
//	else
//		return result; // or throw an exception
//
//	result.resize(count);
//
//	for (int i = count - 1; i > 0; --i)
//	{
//		result[i] = (char)(0x80 | (cp & 0x3F));
//		cp >>= 6;
//	}
//
//	for (int i = 0; i < count; ++i)
//		cp |= (1 << (7 - i));
//
//	result[0] = (char)cp;
//
//	return result;
//}
//
//bool MapManager::decodeUni()
//{
//	// unicode-escape decoding
//	std::string::size_type startIdx = 0;
//	do
//	{
//		startIdx = m_json.find("\\u", startIdx);
//		if (startIdx == std::string::npos) break;
//
//		std::string::size_type endIdx = m_json.find_first_not_of("0123456789abcdefABCDEF", startIdx + 2);
//		if (endIdx == std::string::npos) break;
//
//		std::string tmpStr = m_json.substr(startIdx + 2, endIdx - (startIdx + 2));
//		std::istringstream iss(tmpStr);
//
//		uint32_t cp;
//		if (iss >> std::hex >> cp)
//		{
//			std::string utf8 = to_utf8(cp);
//			m_json.replace(startIdx, 2 + tmpStr.length(), utf8);
//			startIdx += utf8.length();
//		}
//		else
//			startIdx += 2;
//	} while (true);
//
//	return true;
//}

bool MapManager::utf8to16(const char* utf8, std::wstring& utf16)
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

bool MapManager::downloadMap(double lat, double lon, double radius)
{
	const std::string url_middle = ":21500/wgs/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);
	
	return query2server(url);
}

bool MapManager::downloadMap(ID node_id, double radius)
{
	const std::string url_middle = ":21500/routing_node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadMap(cv::Point2i tile)
{
	const std::string url_middle = ":21500/tile/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

bool MapManager::parseMap(const char* json)
{
	Document document;
	document.Parse(json);

	if(!document.IsObject()) return false;
	const Value& features = document["features"];
	if(!features.IsArray()) return false;

	std::vector<EdgeTemp> temp_edge;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if(!feature.IsObject()) return false;
		if(!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if(!properties.IsObject()) return false;
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
		if(!feature.IsObject()) return false;
		if(!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if(!properties.IsObject()) return false;
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
			if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
			{
				node.lon = properties["latitude"].GetDouble();
				node.lat = properties["longitude"].GetDouble();
			}
			else
			{
				node.lat = properties["latitude"].GetDouble();
				node.lon = properties["longitude"].GetDouble();
			}

			const Value& edge_ids = properties["edge_ids"];

			for (Value::ConstValueIterator edge_id = edge_ids.Begin(); edge_id != edge_ids.End(); ++edge_id)
			{
				ID id = edge_id->GetUint64();
				std::vector<EdgeTemp>::iterator itr = std::find_if(temp_edge.begin(), temp_edge.end(), [id](EdgeTemp e) -> bool { return e.id == id; });
				if (itr != temp_edge.end())
					itr->node_ids.push_back(node.id);
				//#ifdef _DEBUG
				//				else
				//					fprintf(stdout, "%d %s\n", ++numNonEdges, "<=======================the number of the edge_ids without edgeinfo"); // the number of the edge_ids without edgeinfo
				//#endif
			}
			m_map->addNode(node);
			//#ifdef _DEBUG
			//			fprintf(stdout, "%d\n", i + 1); // the number of nodes
			//#endif
		}
	}

	for (std::vector<EdgeTemp>::iterator it = temp_edge.begin(); it < temp_edge.end(); it++)
	{
		for (auto i = (it->node_ids).begin(); i < (it->node_ids).end(); i++)
		{
			for (auto j = i; j < it->node_ids.end(); j++)
			{
				if (i == j) continue;
				m_map->addEdge(*i, *j, Edge(it->id, it->length, it->type));
				//m_map.addEdge(*j, *i, Edge(it->id, it->length, it->type));
//#ifdef _DEBUG
//					fprintf(stdout, "%d %s\n", ++numEdges, "<=======================the number of edges"); // the number of edges
//#endif
			}
		}
	}

	return true;
}
//
//bool MapManager::loadMap(double lat, double lon, double radius)
//{
//	if (m_isMap)
//	{
//		delete m_map;
//		m_isMap = false;
//
//		m_path.pts.clear();
//	}
//	m_map = new Map();
//	m_isMap = true;
//    //m_map->nodes.clear();
//	m_json = "";
//
//	// by communication
//	bool ok = downloadMap(lat, lon, radius); // 1000.0);
//	if (!ok) return false;
//	//decodeUni();
//	const char* json = m_json.c_str();
////#ifdef _DEBUG
////	fprintf(stdout, "%s\n", json);
////#endif
//
//	return parseMap(json);
//}

Map& MapManager::getMap()
{
	return *m_map;
}

bool MapManager::getMap(double lat, double lon, double radius, Map& map)
{
	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		//m_path.pts.clear();
		//lookup_path.clear();
	}
	m_map = new Map();
	m_isMap = true;
	//m_map->nodes.clear();
	m_json = "";

	// by communication
	bool ok = downloadMap(lat, lon, radius);
	if (!ok) return false;
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif

	ok = parseMap(json);
	if (!ok) return false;
	map = getMap();

	return true;
}

bool MapManager::getMap(ID node_id, double radius, Map& map)
{
	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		//m_path.pts.clear();
		//lookup_path.clear();
	}
	m_map = new Map();
	m_isMap = true;
	//m_map->nodes.clear();
	m_json = "";

	// by communication
	bool ok = downloadMap(node_id, radius);
	if (!ok) return false;
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif

	ok = parseMap(json);
	if (!ok) return false;
	map = getMap();

	return true;
}

bool MapManager::getMap(cv::Point2i tile, Map& map)
{
	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		//m_path.pts.clear();
		//lookup_path.clear();
	}
	m_map = new Map();
	m_isMap = true;
	//m_map->nodes.clear();
	m_json = "";

	// by communication
	bool ok = downloadMap(tile);
	if (!ok) return false;
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif

	ok = parseMap(json);
	if (!ok) return false;
	map = getMap();

	return true;
}

bool MapManager::getMap(Path path, Map& map, double alpha)
{
	/*double lat = 36.38;
	double lon = 127.373;
	double r = 1000.0;


	loadMap(lat, lon, r);

	return getMap();*/

	std::vector<double> lats, lons;

	if(path.pts.size() == 0) return false;

	for (std::vector<PathElement>::iterator it = path.pts.begin(); it < path.pts.end(); it++)
	{
		auto found = lookup_path.find(it->node_id);
		if (found == lookup_path.end()) return false;
		
		// swapped lat and lon
		if ((found->second.lat) > (found->second.lon))
		{
			lats.push_back(found->second.lon);
			lons.push_back(found->second.lat);
			continue;
		}

		lats.push_back(found->second.lat);
		lons.push_back(found->second.lon);
	}

	double min_lat = *min_element(lats.begin(), lats.end());
	double max_lat = *max_element(lats.begin(), lats.end());
	double min_lon = *min_element(lons.begin(), lons.end());
	double max_lon = *max_element(lons.begin(), lons.end());
	
	UTMConverter utm_conv;
	Point2 min_metric = utm_conv.toMetric(LatLon(min_lat, min_lon));
	Point2 max_metric = utm_conv.toMetric(LatLon(max_lat, max_lon));
	double dist_metric = sqrt(pow((max_metric.x - min_metric.x), 2) + pow((max_metric.y - min_metric.y), 2));
	double center_lat = (min_lat + max_lat) / 2;
	double center_lon = (min_lon + max_lon) / 2;


	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		//m_path.pts.clear();
		//lookup_path.clear();
	}
	m_map = new Map();
	m_isMap = true;
	//m_map->nodes.clear();
	m_json = "";

	// by communication
	bool ok = downloadMap(center_lat, center_lon, (dist_metric / 2) + alpha);
	if (!ok) return false;

	const char* json = m_json.c_str();
	ok = parseMap(json);
	if (!ok) return false;

	map = getMap();

	return true;
}

bool MapManager::getMap_expansion(Path path, Map& map, double alpha)
{
	/*double lat = 36.38;
	double lon = 127.373;
	double r = 1000.0;


	loadMap(lat, lon, r);

	return getMap();*/

	std::vector<double> lats, lons;

	if (path.pts.size() == 0) return false;

	for (std::vector<PathElement>::iterator it = path.pts.begin(); it < path.pts.end(); it++)
	{
		auto found = lookup_path.find(it->node_id);
		if (found == lookup_path.end()) return false;

		// swapped lat and lon
		if ((found->second.lat) > (found->second.lon))
		{
			lats.push_back(found->second.lon);
			lons.push_back(found->second.lat);
			continue;
		}

		lats.push_back(found->second.lat);
		lons.push_back(found->second.lon);
	}

	double min_lat = *min_element(lats.begin(), lats.end());
	double max_lat = *max_element(lats.begin(), lats.end());
	double min_lon = *min_element(lons.begin(), lons.end());
	double max_lon = *max_element(lons.begin(), lons.end());

	// auto topological map expansion mode
	std::vector<double> lats_map, lons_map;
	for (std::vector<Node>::iterator it = m_map->nodes.begin(); it < m_map->nodes.end(); it++)
	{
		// swapped lat and lon
		if ((it->lat) > (it->lon))
		{
			lats_map.push_back(it->lon);
			lons_map.push_back(it->lat);
			continue;
		}

		lats_map.push_back(it->lat);
		lons_map.push_back(it->lon);
	}
	double min_lat_map = *min_element(lats_map.begin(), lats_map.end());
	double max_lat_map = *max_element(lats_map.begin(), lats_map.end());
	double min_lon_map = *min_element(lons_map.begin(), lons_map.end());
	double max_lon_map = *max_element(lons_map.begin(), lons_map.end());
	if (min_lat > min_lat_map&& max_lat < max_lat_map && min_lon > min_lon_map&& max_lon < max_lon_map)
		return true;

	UTMConverter utm_conv;
	Point2 min_metric = utm_conv.toMetric(LatLon(min_lat, min_lon));
	Point2 max_metric = utm_conv.toMetric(LatLon(max_lat, max_lon));
	double dist_metric = sqrt(pow((max_metric.x - min_metric.x), 2) + pow((max_metric.y - min_metric.y), 2));
	double center_lat = (min_lat + max_lat) / 2;
	double center_lon = (min_lon + max_lon) / 2;


	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		//m_path.pts.clear();
		//lookup_path.clear();
	}
	m_map = new Map();
	m_isMap = true;
	//m_map->nodes.clear();
	m_json = "";

	// by communication
	bool ok = downloadMap(center_lat, center_lon, (dist_metric / 2) + alpha);
	if (!ok) return false;

	const char* json = m_json.c_str();
	ok = parseMap(json);
	if (!ok) return false;

	map = getMap();

	return true;
}

std::vector<Node> MapManager::getMap_junction(LatLon cur_latlon, int top_n)
{
	std::vector<Node> node_vec;

	UTMConverter utm_conv;
	Point2 cur_metric = utm_conv.toMetric(cur_latlon);
	Point2 node_metric;
	double dist_metric;
	/** A hash table for finding junction nodes by distance */
	std::map<double, Node> lookup_junc_dist;

	for (std::vector<Node>::iterator it = m_map->nodes.begin(); it != m_map->nodes.end(); ++it)
	{
		if(it->type == Node::NODE_JUNCTION)
		{
			node_metric = utm_conv.toMetric(LatLon(it->lat, it->lon));
			dist_metric = sqrt(pow((cur_metric.x - node_metric.x), 2) + pow((cur_metric.y - node_metric.y), 2));
			lookup_junc_dist.insert(std::make_pair(dist_metric, *it));
		}
	}
	
	int num = 0;
	for (std::map<double, Node>::iterator it = lookup_junc_dist.begin(); it != lookup_junc_dist.end(); ++it)
	{
		if (num >= top_n)
			break;		
		num++;

		node_vec.push_back(it->second);
	}

	return node_vec;
}

bool MapManager::downloadPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, int num_paths)
{
	const std::string url_middle = ":20005/"; // routing server (paths)
	std::string url = "http://" + m_ip + url_middle + std::to_string(start_lat) + "/" + std::to_string(start_lon) + "/" + std::to_string(start_floor) + "/" + std::to_string(dest_lat) + "/" + std::to_string(dest_lon) + "/" + std::to_string(dest_floor) + "/" + std::to_string(num_paths);

	return query2server(url);
}

bool MapManager::parsePath(const char* json)
{
	Document document;
	document.Parse(json);

	if(!document[0].IsObject()) return false;
	const Value& features = document[0]["features"];
	if(!features.IsArray()) return false;

	Node node;
	Edge edge;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if(!feature.IsObject()) return false;
		if(!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if(!properties.IsObject()) return false;
		std::string name = properties["name"].GetString();
		if (i % 2 == 0)	// node
		{
			if (!(name == "Node" || name == "node")) return false;
			//node = new Node();
			node.id = properties["id"].GetUint64();
			//switch (properties["type"].GetInt())
			//{
			///** Basic node */
			//case 0: node.type = Node::NODE_BASIC; break;
			///** Junction node (e.g. intersecting point, corner point, and end point of the road) */
			//case 1: node.type = Node::NODE_JUNCTION; break;
			///** Door node (e.g. exit and entrance) */
			//case 2: node.type = Node::NODE_DOOR; break;
			///** Elevator node */
			//case 3: node.type = Node::NODE_ELEVATOR; break;
			///** Escalator node */
			//case 4: node.type = Node::NODE_ESCALATOR; break;
			//}
			//node.floor = properties["floor"].GetInt();

			// swapped lat and lon
			if ((properties["latitude"].GetDouble()) > (properties["longitude"].GetDouble()))
			{
				node.lon = properties["latitude"].GetDouble();
				node.lat = properties["longitude"].GetDouble();
			}
			else
			{
				node.lat = properties["latitude"].GetDouble();
				node.lon = properties["longitude"].GetDouble();
			}

			const Value& edge_ids = properties["edge_ids"];

			/*for (Value::ConstValueIterator edge_id = edge_ids.Begin(); edge_id != edge_ids.End(); ++edge_id)
			{
				node.edge_ids.push_back(edge_id->GetUint64());
			}*/

			if (!((i + 1) < features.Size()))
			{
				edge.id = 0;
				m_path.pts.push_back(PathElement(node.id, edge.id));
				lookup_path.insert(std::make_pair(node.id, LatLon(node.lat, node.lon)));
			}
		}
		else			// edge
		{
			if(!(name == "Edge" || name == "edge")) return false;
			//edge = new Edge();
			edge.id = properties["id"].GetUint64();
			//switch (properties["type"].GetInt())
			//{
			///** Sidewalk */
			//case 0: edge.type = Edge::EDGE_SIDEWALK; break;
			///** General road (e.g. roads shared by pedestrians and cars, street, alley, corridor, ...) */
			//case 1: edge.type = Edge::EDGE_ROAD; break;
			///** Crosswalk */
			//case 2: edge.type = Edge::EDGE_CROSSWALK; break;
			///** Elevator section */
			//case 3: edge.type = Edge::EDGE_ELEVATOR; break;
			///** Escalator section */
			//case 4: edge.type = Edge::EDGE_ESCALATOR; break;
			///** Stair section */
			//case 5: edge.type = Edge::EDGE_STAIR; break;
			//}
			//edge.length = properties["length"].GetDouble();

			m_path.pts.push_back(PathElement(node.id, edge.id));
			lookup_path.insert(std::make_pair(node.id, LatLon(node.lat, node.lon)));
		}
	}

	return true;
}

bool MapManager::generatePath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, int num_paths)
{
	/*UTMConverter utm_conv;
	Point2 start_metric = utm_conv.toMetric(LatLon(start_lat, start_lon));
	Point2 dest_metric = utm_conv.toMetric(LatLon(dest_lat, dest_lon));
	double dist_metric = sqrt(pow((dest_metric.x - start_metric.x), 2) + pow((dest_metric.y - start_metric.y), 2));
	double alpha = 50;*/

	//double center_lat = (start_lat + dest_lat) / 2;
	//double center_lon = (start_lon + dest_lon) / 2;
	//bool ok = loadMap(center_lat, center_lon, 200);//(dist_metric / 2) + alpha);
	//if (!ok) return false;

	m_path.pts.clear();
	lookup_path.clear();
	m_json = "";

	// by communication
	bool ok = downloadPath(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, num_paths);
	if (!ok) return false;
	//decodeUni();
	if (m_json == "[]\n" || m_json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		ok = downloadPath(round(start_lat * 1000) / 1000, round(start_lon * 1000) / 1000, start_floor, round(dest_lat * 1000) / 1000, round(dest_lon * 1000) / 1000, dest_floor, num_paths);
		if (m_json == "[]\n" || m_json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
		{
			std::cout << "Invalid latitude or longitude!!" << std::endl;
			return false;
		}
		if (!ok) return false;
	}
	
	const char* json = m_json.c_str();

//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	ok = parsePath(json);
	if (!ok) return false;

	Path path = m_path;
	Map map;
	ok = getMap(path, map);
	if (!ok) return false;

	/*m_path.pts.clear();
	for (size_t i = 0; i < path.pts.size(); i++)
	{
		PathElement p;
		p.node = m_map->findNode(path.pts[i].node->id);
		
		if ((i + 1) < path.pts.size())
		{
			if ((i + 1) < path.pts.size()) p.edge = m_map->findEdge(path.pts[i].edge->id);
		}
		else p.edge = nullptr;
		m_path.pts.push_back(p);

		delete path.pts[i].node;
		delete path.pts[i].edge;
	}*/

	return true;
}

bool MapManager::generatePath_expansion(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, int num_paths)
{
	/*UTMConverter utm_conv;
	Point2 start_metric = utm_conv.toMetric(LatLon(start_lat, start_lon));
	Point2 dest_metric = utm_conv.toMetric(LatLon(dest_lat, dest_lon));
	double dist_metric = sqrt(pow((dest_metric.x - start_metric.x), 2) + pow((dest_metric.y - start_metric.y), 2));
	double alpha = 50;*/

	//double center_lat = (start_lat + dest_lat) / 2;
	//double center_lon = (start_lon + dest_lon) / 2;
	//bool ok = loadMap(center_lat, center_lon, 200);//(dist_metric / 2) + alpha);
	//if (!ok) return false;

	m_path.pts.clear();
	lookup_path.clear();
	m_json = "";

	// by communication
	bool ok = downloadPath(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, num_paths);
	if (!ok) return false;
	//decodeUni();
	if (m_json == "[]\n" || m_json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		ok = downloadPath(round(start_lat * 1000) / 1000, round(start_lon * 1000) / 1000, start_floor, round(dest_lat * 1000) / 1000, round(dest_lon * 1000) / 1000, dest_floor, num_paths);
		if (m_json == "[]\n" || m_json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
		{
			std::cout << "Invalid latitude or longitude!!" << std::endl;
			return false;
		}
		if (!ok) return false;
	}

	const char* json = m_json.c_str();

	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif
	ok = parsePath(json);
	if (!ok) return false;

	Path path = m_path;
	Map map;
	ok = getMap_expansion(path, map);	// The On of auto topological map expansion mode
	if (!ok) return false;

	/*m_path.pts.clear();
	for (size_t i = 0; i < path.pts.size(); i++)
	{
		PathElement p;
		p.node = m_map->findNode(path.pts[i].node->id);

		if ((i + 1) < path.pts.size())
		{
			if ((i + 1) < path.pts.size()) p.edge = m_map->findEdge(path.pts[i].edge->id);
		}
		else p.edge = nullptr;
		m_path.pts.push_back(p);

		delete path.pts[i].node;
		delete path.pts[i].edge;
	}*/

	return true;
}

Path MapManager::getPath()
{
	return m_path;
}

bool MapManager::getPath(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, int num_paths)
{
	bool ok = generatePath(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, num_paths);
	if (!ok) return false;

	path = getPath();

	return true;
}

bool MapManager::getPath_expansion(double start_lat, double start_lon, int start_floor, double dest_lat, double dest_lon, int dest_floor, Path& path, int num_paths)
{
	bool ok = generatePath_expansion(start_lat, start_lon, start_floor, dest_lat, dest_lon, dest_floor, num_paths);
	if (!ok) return false;

	path = getPath();

	return true;
}

bool MapManager::getPath(const char* filename, Path& path)
{
	m_path.pts.clear();
	lookup_path.clear();
	m_json = "";

	// Convert JSON document to string
	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line, text;
	while (std::getline(is, line))
	{
		text += line + "\n";
	}
	const char* json = text.c_str();
	is.close();

	bool ok = parsePath(json);
	if (!ok)
	{
		m_path.pts.clear();
		lookup_path.clear();
	
		return false;
	}

	path = getPath();

	return true;
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

bool MapManager::downloadPOI_poi(ID poi_id, double radius)
{
	const std::string url_middle = ":21502/node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(poi_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::parsePOI(const char* json)
{
	Document document;
	document.Parse(json);

	if(!document.IsObject()) return false;
	const Value& features = document["features"];
	if(!features.IsArray()) return false;

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		if(!feature.IsObject()) return false;
		if(!feature.HasMember("properties")) return false;
		const Value& properties = feature["properties"];
		if(!properties.IsObject()) return false;
		POI poi;
		poi.id = properties["id"].GetUint64();
		const char* utf8 = properties["name"].GetString();
		utf8to16(utf8, poi.name);
		poi.floor = properties["floor"].GetInt();
		poi.lat = properties["latitude"].GetDouble();
		poi.lon = properties["longitude"].GetDouble();

		m_map->addPOI(poi);
	}

	return true;
}

std::vector<POI>& MapManager::getPOI()
{
	return m_map->pois;
}

bool MapManager::getPOI(double lat, double lon, double radius, std::vector<POI>& poi_vec)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI(lat, lon, radius);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	if (json == "[]\n" || json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		std::cout << "Invalid latitude or longitude!!" << std::endl;
		return false;
	}

	bool ok = parsePOI(json);
	if (!ok)
	{
		m_map->pois.clear();

		return false;
	}

	poi_vec = getPOI();

	return true;
}

 bool MapManager::getPOI(ID node_id, double radius, std::vector<POI>& poi_vec)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI(node_id, radius);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	if (json == "[]\n" || json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		std::cout << "Invalid node ID!!" << std::endl;
		return false;
	}

	bool ok = parsePOI(json);
	if (!ok)
	{
		m_map->pois.clear();

		return false;
	}

	poi_vec = getPOI();

	return true;
}

bool MapManager::getPOI(cv::Point2i tile, std::vector<POI>& poi_vec)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI(tile);
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif
	if (json == "[]\n" || json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		std::cout << "Invalid map tile!!" << std::endl;
		return false;
	}

	bool ok = parsePOI(json);
	if (!ok)
	{
		m_map->pois.clear();

		return false;
	}

	poi_vec = getPOI();

	return true;
}

//POI MapManager::getPOI(ID poi_id, LatLon latlon, double radius)
//{
//	std::vector<POI> poi_vec;
//	bool ok = getPOI(latlon.lat, latlon.lon, radius, poi_vec);
//	if (!ok)
//		return POI();
//	for (std::vector<POI>::iterator it = m_map->pois.begin(); it != m_map->pois.end(); ++it)
//	{
//		if (it->id == poi_id)
//			return *it;
//	}
//
//	return POI();
//}

//POI MapManager::getPOI(ID poi_id)
//{
//	if(m_map->pois.size() == 0) 
//		return POI();
//	   	
//	auto found = lookup_pois_id.find(poi_id);
//	if (found == lookup_pois_id.end())
//		return POI();
//
//	return getPOI(poi_id, found->second, 10.0);
//}
std::vector<POI> MapManager::getPOI(ID poi_id, double radius)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI_poi(poi_id, radius);
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif
	if (json == "[]\n" || json == "{\"type\": \"FeatureCollection\", \"features\": []}\n")
	{
		std::cout << "Invalid POI IDs!!" << std::endl;
		return std::vector<POI>();
	}

	bool ok = parsePOI(json);
	if (!ok)
	{
		m_map->pois.clear();

		return std::vector<POI>();
	}
	
	return getPOI();
}

std::vector<POI> MapManager::getPOI(const std::string poi_name, LatLon latlon, double radius)
{	
	std::vector<POI> poi_vec;
	bool ok = getPOI(latlon.lat, latlon.lon, radius, poi_vec);
	if (!ok)
		return std::vector<POI>();
	poi_vec.clear();
	std::wstring name;
	utf8to16(poi_name.c_str(), name);
	for (std::vector<POI>::iterator it = m_map->pois.begin(); it != m_map->pois.end(); ++it)
	{		
		if (it->name == name)
			poi_vec.push_back(*it);
	}	

	return poi_vec;
}

std::vector<POI> MapManager::getPOI_sorting(const std::string poi_name, LatLon latlon, double radius, LatLon cur_latlon)
{
	std::vector<POI> poi_vec;
	bool ok = getPOI(latlon.lat, latlon.lon, radius, poi_vec);
	if (!ok)
		return std::vector<POI>();
	poi_vec.clear();
	std::wstring name;
	utf8to16(poi_name.c_str(), name);

	UTMConverter utm_conv;
	Point2 cur_metric = utm_conv.toMetric(cur_latlon);
	Point2 poi_metric;
	double dist_metric;
	/** A hash table for finding POIs by distance */
	std::map<double, POI> lookup_pois_dist;
	
	for (std::vector<POI>::iterator it = m_map->pois.begin(); it != m_map->pois.end(); ++it)
	{
		if (it->name == name)
		{
			poi_metric = utm_conv.toMetric(LatLon(it->lat, it->lon));
			dist_metric = sqrt(pow((cur_metric.x - poi_metric.x), 2) + pow((cur_metric.y - poi_metric.y), 2));
			lookup_pois_dist.insert(std::make_pair(dist_metric, *it));
		}
	}

	for (std::map<double, POI>::iterator it = lookup_pois_dist.begin(); it != lookup_pois_dist.end(); ++it)
	{
		poi_vec.push_back(it->second);
	}

	return poi_vec;
}

std::vector<POI> MapManager::getPOI(const std::string poi_name)
{
	std::wstring name;
	utf8to16(poi_name.c_str(), name);
	auto found = lookup_pois_name.find(name);
	if (found == lookup_pois_name.end()) 
		return std::vector<POI>();

	return getPOI(poi_name, found->second, 10.0);
}

std::vector<POI> MapManager::getPOI_sorting(const std::string poi_name, LatLon cur_latlon)
{
	std::wstring name;
	utf8to16(poi_name.c_str(), name);
	auto found = lookup_pois_name.find(name);
	if (found == lookup_pois_name.end())
		return std::vector<POI>();

	return getPOI_sorting(poi_name, found->second, 10.0, cur_latlon);
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


bool MapManager::downloadStreetView_sv(ID sv_id, double radius)
{
	const std::string url_middle = ":21501/node/";
	std::string url = "http://" + m_ip + url_middle + std::to_string(sv_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::parseStreetView(const char* json)
{
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
		sv.lat = properties["latitude"].GetDouble();
		sv.lon = properties["longitude"].GetDouble();

		m_map->addView(sv);
	}

	return true;
}

std::vector<StreetView> MapManager::getStreetView()
{
	return m_map->views;
}

bool MapManager::getStreetView(double lat, double lon, double radius, std::vector<StreetView>& sv_vec)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView(lat, lon, radius);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parseStreetView(json);
	if (!ok)
	{
		m_map->views.clear();

		return false;
	}

	sv_vec = getStreetView();

	return true;
}

bool MapManager::getStreetView(ID node_id, double radius, std::vector<StreetView>& sv_vec)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView(node_id, radius);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parseStreetView(json);
	if (!ok)
	{
		m_map->views.clear();

		return false;
	}

	sv_vec = getStreetView();

	return true;
}

bool MapManager::getStreetView(cv::Point2i tile, std::vector<StreetView>& sv_vec)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView(tile);
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif
	bool ok = parseStreetView(json);
	if (!ok)
	{
		m_map->views.clear();

		return false;
	}

	sv_vec = getStreetView();

	return true;
}

//StreetView MapManager::getStreetView(ID sv_id, LatLon latlon, double radius)
//{
//	std::vector<StreetView> sv_vec;
//	bool ok = getStreetView(latlon.lat, latlon.lon, radius, sv_vec);
//	if (!ok)
//		return StreetView();
//	for (std::vector<StreetView>::iterator it = m_map->views.begin(); it != m_map->views.end(); ++it)
//	{
//		if (it->id == sv_id)
//			return *it;
//	}
//
//	return StreetView();
//}

//StreetView MapManager::getStreetView(ID sv_id)
//{
//	if (m_map->views.size() == 0)
//		return StreetView();
//
//	auto found = lookup_svs.find(sv_id);
//	if (found == lookup_svs.end())
//		return StreetView();
//
//	return getStreetView(sv_id, found->second, 10.0);
//}
std::vector<StreetView> MapManager::getStreetView(ID sv_id, double radius)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView_sv(sv_id, radius);
	//decodeUni();
	const char* json = m_json.c_str();
	//#ifdef _DEBUG
	//	fprintf(stdout, "%s\n", json);
	//#endif
	bool ok = parseStreetView(json);
	if (!ok)
	{
		m_map->views.clear();

		return std::vector<StreetView>();
	}

	return getStreetView();
}

size_t MapManager::writeImage_callback(char* ptr, size_t size, size_t nmemb, void* userdata)
{
	std::vector<uchar>* stream = (std::vector<uchar>*)userdata;
	size_t count = size * nmemb;
	stream->insert(stream->end(), ptr, ptr + count);
	return count;
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

cv::Mat MapManager::downloadStreetViewImage(ID sv_id, const std::string cubic, int timeout, const std::string url_middle)
{
	//const std::string url_middle = ":10000/";
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

bool MapManager::getStreetViewImage(ID sv_id, cv::Mat& sv_image, std::string cubic, int timeout)
{
	if (!(cubic == "f" || cubic == "b" || cubic == "l" || cubic == "r" || cubic == "u" || cubic == "d"))
		cubic = "";

	sv_image = downloadStreetViewImage(sv_id, cubic, timeout);

	if (m_portErr == true)
	{
		const std::string url_middle = ":10001/";
		sv_image = downloadStreetViewImage(sv_id, cubic, timeout, url_middle);
		m_portErr = false;
	}

	if (sv_image.empty())	return false;

	return true;
}

} // End of 'dg'

