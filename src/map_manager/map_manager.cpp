#include "map_manager.hpp"

namespace dg
{

int MapManager::lat2tiley(double lat, int z)
{
	double latrad = lat * M_PI / 180.0;
	return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)));
}

int MapManager::lon2tilex(double lon, int z)
{
	return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
}

double MapManager::tiley2lat(int y, int z)
{
	double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
	return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

double MapManager::tilex2lon(int x, int z)
{
	return x / (double)(1 << z) * 360.0 - 180;
}

cv::Point2i MapManager::latlon2xy(double lat, double lon, int z)
{
	return cv::Point2i(lon2tilex(lon, z), lat2tiley(lat, z));
}

bool MapManager::initialize()
{

	return true;
}

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

bool MapManager::downloadMap(double lat, double lon, double radius, const std::string ip)
{
	const std::string url_middle = ":21500/wgs/";
	std::string url = "http://" + ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);
	
	return query2server(url);
}

bool MapManager::downloadMap(ID node_id, double radius, const std::string ip)
{
	const std::string url_middle = ":21500/node/";
	std::string url = "http://" + ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadMap(cv::Point2i tile, const std::string ip)
{
	const std::string url_middle = ":21500/tile/";
	std::string url = "http://" + ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

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
			node.lat = properties["latitude"].GetDouble();
			node.lon = properties["longitude"].GetDouble();
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

bool MapManager::loadMap(double lat, double lon, double radius, const std::string ip)
{
	if (m_isMap)
	{
		delete m_map;
		m_isMap = false;

		m_path.pts.clear();
	}
	m_map = new Map();
	m_isMap = true;
    //m_map->nodes.clear();
	m_json = "";

	// by communication
	downloadMap(lat, lon, radius, ip); // 1000.0);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif

	return parseMap(json);
}

Map& MapManager::getMap(double lat, double lon, double radius, const std::string ip)
{
	loadMap(lat, lon, radius, ip);

	return getMap();
}

Map& MapManager::getMap(Path path, const std::string ip)
{
	/*double lat = 36.38;
	double lon = 127.373;
	double r = 1000.0;


	loadMap(lat, lon, r);

	return getMap();*/

	std::vector<double> lats, lons;

	if(path.pts.size() == 0) return getMap();
	for (std::vector<PathElement>::iterator it = path.pts.begin(); it < path.pts.end(); it++)
	{
		// swapped lat and lon
		if ((it->node->lat) > (it->node->lon))
		{
			lats.push_back(it->node->lon);
			lons.push_back(it->node->lat);
			continue;
		}
		lats.push_back(it->node->lat);
		lons.push_back(it->node->lon);

		/*lats.push_back(it->node->lat);
		lons.push_back(it->node->lon);*/
	}

	double min_lat = *min_element(lats.begin(), lats.end());
	double max_lat = *max_element(lats.begin(), lats.end());
	double min_lon = *min_element(lons.begin(), lons.end());
	double max_lon = *max_element(lons.begin(), lons.end());

	UTMConverter utm_conv;
	Point2 min_metric = utm_conv.toMetric(LatLon(min_lat, min_lon));
	Point2 max_metric = utm_conv.toMetric(LatLon(max_lat, max_lon));
	double dist_metric = sqrt(pow((max_metric.x - min_metric.x), 2) + pow((max_metric.y - min_metric.y), 2));
	double alpha = 20;
	double center_lat = (min_lat + max_lat) / 2;
	double center_lon = (min_lon + max_lon) / 2;

	loadMap(center_lat, center_lon, (dist_metric / 2) + alpha, ip);

	return getMap();
}

Map& MapManager::getMap()
{
	return *m_map;
}

bool MapManager::downloadPath(double start_lat, double start_lon, double dest_lat, double dest_lon, const std::string ip, int num_paths)
{
	const std::string url_middle = ":20005/"; // routing server (paths)
	std::string url = "http://" + ip + url_middle + std::to_string(start_lat) + "/" + std::to_string(start_lon) + "/" + std::to_string(dest_lat) + "/" + std::to_string(dest_lon) + "/" + std::to_string(num_paths);

	return query2server(url);
}

bool MapManager::parsePath(const char* json)
{
	Document document;
	document.Parse(json);

	if(!document[0].IsObject()) return false;
	const Value& features = document[0]["features"];
	if(!features.IsArray()) return false;

	Node* node = nullptr;
	Edge* edge = nullptr;
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
			node = new Node();
			node->id = properties["id"].GetUint64();
			//switch (properties["type"].GetInt())
			//{
			///** Basic node */
			//case 0: node->type = Node::NODE_BASIC; break;
			///** Junction node (e.g. intersecting point, corner point, and end point of the road) */
			//case 1: node->type = Node::NODE_JUNCTION; break;
			///** Door node (e.g. exit and entrance) */
			//case 2: node->type = Node::NODE_DOOR; break;
			///** Elevator node */
			//case 3: node->type = Node::NODE_ELEVATOR; break;
			///** Escalator node */
			//case 4: node->type = Node::NODE_ESCALATOR; break;
			//}
			//node->floor = properties["floor"].GetInt();
			node->lat = properties["latitude"].GetDouble();
			node->lon = properties["longitude"].GetDouble();
		
			if (!((i + 1) < features.Size()))
			{
				edge = nullptr;
				m_path.pts.push_back(PathElement(node, edge));
			}
		}
		else			// edge
		{
			if(!(name == "Edge" || name == "edge")) return false;
			edge = new Edge();
			edge->id = properties["id"].GetUint64();
			//switch (properties["type"].GetInt())
			//{
			///** Sidewalk */
			//case 0: edge->type = Edge::EDGE_SIDEWALK; break;
			///** General road (e.g. roads shared by pedestrians and cars, street, alley, corridor, ...) */
			//case 1: edge->type = Edge::EDGE_ROAD; break;
			///** Crosswalk */
			//case 2: edge->type = Edge::EDGE_CROSSWALK; break;
			///** Elevator section */
			//case 3: edge->type = Edge::EDGE_ELEVATOR; break;
			///** Escalator section */
			//case 4: edge->type = Edge::EDGE_ESCALATOR; break;
			///** Stair section */
			//case 5: edge->type = Edge::EDGE_STAIR; break;
			//}
			//edge->length = properties["length"].GetDouble();

			m_path.pts.push_back(PathElement(node, edge));
		}
	}

	return true;
}

bool MapManager::generatePath(double start_lat, double start_lon, double dest_lat, double dest_lon, const std::string ip, int num_paths)
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
	m_json = "";

	// by communication
	bool ok = downloadPath(start_lat, start_lon, dest_lat, dest_lon, ip, num_paths);
	if (!ok) return false;
	//decodeUni();
	const char* json = m_json.c_str();

//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif

	ok = parsePath(json);
	if (!ok) return false;

	Path path = m_path;
	getMap(path);

	m_path.pts.clear();
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
	}

	return true;
}

Path MapManager::getPath(double start_lat, double start_lon, double dest_lat, double dest_lon, const std::string ip, int num_paths)
{
	generatePath(start_lat, start_lon, dest_lat, dest_lon, ip, num_paths);

	return getPath();
}

Path MapManager::getPath(const char* filename)
{
	m_path.pts.clear();
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
	if (!ok) m_path.pts.clear();

	return getPath();
}

Path MapManager::getPath()
{
	return m_path;
}

bool MapManager::downloadPOI(double lat, double lon, double radius, const std::string ip)
{
	const std::string url_middle = ":21502/wgs/";
	std::string url = "http://" + ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(ID node_id, double radius, const std::string ip)
{
	const std::string url_middle = ":21502/node/";
	std::string url = "http://" + ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(cv::Point2i tile, const std::string ip)
{
	const std::string url_middle = ":21502/tile/";
	std::string url = "http://" + ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

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

		m_map->pois.push_back(poi);
	}

	return true;
}

std::list<POI>& MapManager::getPOI(double lat, double lon, double radius, const std::string ip)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI(lat, lon, radius, ip);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parsePOI(json);
	if (!ok) m_map->pois.clear();

	return getPOI();
}

std::list<POI>& MapManager::getPOI(ID node_id, double radius, const std::string ip)
{
	m_map->pois.clear();
	m_json = "";

	// by communication
	downloadPOI(node_id, radius, ip);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parsePOI(json);
	if (!ok) m_map->pois.clear();

	return getPOI();
}

std::list<POI>& MapManager::getPOI()
{
	return m_map->pois;
}

//std::vector<cv::Point2d> MapManager::getPOIloc(const char* poiname)
//{
//	std::vector<cv::Point2d> points;
//	for (dg::Map::NodeItr node_itr = m_map.getHeadNode(); node_itr != m_map.getTailNode(); ++node_itr)
//	{
//		for (std::vector<std::string>::iterator it = node_itr->data.pois.begin(); it != node_itr->data.pois.end(); ++it)
//		{
//			if (*(it) == poiname)
//			{
//				cv::Point2d point(node_itr->data.lat, node_itr->data.lon);
//				points.push_back(point);
//				break;
//			}
//		}
//	}
//	return points;
//}

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
		sv.id = (ID)properties["id"].GetString();
		std::string name = properties["name"].GetString();
		if (!(name == "streetview" || name == "StreetView")) return false;
		sv.floor = properties["floor"].GetInt();
		sv.date = properties["date"].GetString();
		sv.heading = properties["latitude"].GetDouble();
		sv.lat = properties["latitude"].GetDouble();
		sv.lon = properties["longitude"].GetDouble();

		m_map->views.push_back(sv);
	}

	return true;
}

bool MapManager::downloadStreetView(double lat, double lon, double radius, const std::string ip)
{
	const std::string url_middle = ":21501/wgs/";
	std::string url = "http://" + ip + url_middle + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadStreetView(ID node_id, double radius, const std::string ip)
{
	const std::string url_middle = ":21501/node/";
	std::string url = "http://" + ip + url_middle + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadStreetView(cv::Point2i tile, const std::string ip)
{
	const std::string url_middle = ":21501/tile/";
	std::string url = "http://" + ip + url_middle + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

std::list<StreetView>& MapManager::getStreetView(double lat, double lon, double radius, const std::string ip)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView(lat, lon, radius, ip);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parseStreetView(json);
	if (!ok) m_map->views.clear();

	return getStreetView();
}

std::list<StreetView>& MapManager::getStreetView(ID node_id, double radius, const std::string ip)
{
	m_map->views.clear();
	m_json = "";

	// by communication
	downloadStreetView(node_id, radius, ip);
	//decodeUni();
	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
	bool ok = parseStreetView(json);
	if (!ok) m_map->views.clear();

	return getStreetView();
}

std::list<StreetView>& MapManager::getStreetView()
{
	return m_map->views;
}

} // End of 'dg'

