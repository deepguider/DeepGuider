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
		//struct curl_slist* headers = NULL;

		if (curl)
		{
			curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
			//headers = curl_slist_append(headers, client_id.c_str());
			//headers = curl_slist_append(headers, client_secret.c_str());
			//curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers); /* pass our list of custom made headers */

			std::string response;
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
			curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

			// Perform the request, res will get the return code.
			res = curl_easy_perform(curl);

			// Always cleanup.
			//curl_slist_free_all(headers);
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

bool MapManager::downloadMap(double lat, double lon, double radius)
{
	const std::string url_head = "http://129.254.87.96:21500/wgs/";
	std::string url = url_head + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);
	
	return query2server(url);
}

bool MapManager::downloadMap(ID node_id, double radius)
{
	const std::string url_head = "http://129.254.87.96:21500/node/";
	std::string url = url_head + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadMap(cv::Point2i tile)
{
	const std::string url_head = "http://129.254.87.96:21500/tile/";
	std::string url = url_head + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

// unicode-escape decoding
std::string MapManager::to_utf8(uint32_t cp)
{
	/*
	if using C++11 or later, you can do this:

	std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
	return conv.to_bytes( (char32_t)cp );

	Otherwise...
	*/

	std::string result;

	int count;
	if (cp < 0x0080)
		count = 1;
	else if (cp < 0x0800)
		count = 2;
	else if (cp < 0x10000)
		count = 3;
	else if (cp <= 0x10FFFF)
		count = 4;
	else
		return result; // or throw an exception

	result.resize(count);

	for (int i = count - 1; i > 0; --i)
	{
		result[i] = (char)(0x80 | (cp & 0x3F));
		cp >>= 6;
	}

	for (int i = 0; i < count; ++i)
		cp |= (1 << (7 - i));

	result[0] = (char)cp;

	return result;
}

bool MapManager::decodeUni()
{
	// unicode-escape decoding
	std::string::size_type startIdx = 0;
	do
	{
		startIdx = m_json.find("\\u", startIdx);
		if (startIdx == std::string::npos) break;

		std::string::size_type endIdx = m_json.find_first_not_of("0123456789abcdefABCDEF", startIdx + 2);
		if (endIdx == std::string::npos) break;

		std::string tmpStr = m_json.substr(startIdx + 2, endIdx - (startIdx + 2));
		std::istringstream iss(tmpStr);

		uint32_t cp;
		if (iss >> std::hex >> cp)
		{
			std::string utf8 = to_utf8(cp);
			m_json.replace(startIdx, 2 + tmpStr.length(), utf8);
			startIdx += utf8.length();
		}
		else
			startIdx += 2;
	} while (true);

	return true;
}

bool MapManager::load(double lat, double lon, double radius)
{
    m_map.nodes.clear();
	m_json = "";

	// by communication
	downloadMap(lat, lon, radius); // 1000.0);
	decodeUni();
	const char* json = m_json.c_str();
#ifdef _DEBUG
	fprintf(stdout, "%s\n", json);
#endif
	Document document;
	document.Parse(json);
	
	assert(document.IsObject());
	const Value& features = document["features"];
	assert(features.IsArray());

	std::vector<EdgeTemp> temp_edge;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		std::string name = properties["name"].GetString();
		if (name == "edge") //continue; //TODO
		{
			EdgeTemp edge;
			edge.id = properties["id"].GetUint64();
			switch (properties["type"].GetInt())
			{
				/** Sidewalk */
				case 0: edge.type = ET_SD; break;
				/** Middle road (e.g. lane, ginnel, roads shared by pedestrians and cars, ...) */
				case 1: edge.type = ET_MD; break;
				/** Crosswalk */
				case 2: edge.type = ET_CR; break;
				/** Doorway */
				case 3: edge.type = ET_DR; break;
				/** Elevator section */
				case 4: edge.type = ET_EV; break;
				/** Escalator section */
				case 5: edge.type = ET_ES; break;
			}
			edge.length = properties["length"].GetDouble();
			temp_edge.push_back(edge);
		}
	}

	int numNonEdges = 0;
	int numEdges = 0;
	for (SizeType i = 0; i < features.Size(); i++)
	{
		//document.Parse(json);
		//const Value& features = document["features"];
		const Value& feature = features[i];
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		std::string name = properties["name"].GetString();
		if (name == "Node")
		{
			Node node;
			node.id = properties["id"].GetUint64();
			switch (properties["type"].GetInt())
			{
				/** Basic node */
				case 0: node.type = NT_BS; break;
				/** Junction node (e.g. intersecting point, corner point, and end point of the road) */
				case 1: node.type = NT_JT; break;
				/** Door node (e.g. exit and entrance) */
				case 2: node.type = NT_DR; break;
				/** Elevator node */
				case 3: node.type = NT_EV; break;
				/** Escalator node */
				case 4: node.type = NT_ES; break;
			}
			node.floor = properties["floor"].GetInt();
			node.lat = properties["latitude"].GetDouble();
			node.lon = properties["longitude"].GetDouble();
			const Value& edge_ids = properties["edge_ids"];
			
			for (Value::ConstValueIterator edge_id = edge_ids.Begin(); edge_id != edge_ids.End(); ++edge_id)
			{
				ID id = edge_id->GetUint64();
				std::vector<EdgeTemp>::iterator itr = std::find_if(temp_edge.begin(), temp_edge.end(), [id](EdgeTemp e) -> bool { return e.id == id; });
				if(itr != temp_edge.end())
					itr->node_ids.push_back(node.id);
#ifdef _DEBUG
				else
					fprintf(stdout, "%d %s\n", ++numNonEdges, "<=======================the number of the edge_ids without edgeinfo"); // the number of the edge_ids without edgeinfo
#endif
			}
			m_map.addNode(node);
#ifdef _DEBUG
			fprintf(stdout, "%d\n", i + 1); // the number of nodes
#endif
		}
	}

	for (std::vector<EdgeTemp>::iterator it = temp_edge.begin(); it < temp_edge.end(); it++)
	{
		for (auto i = (it->node_ids).begin(); i < (it->node_ids).end(); i++)
		{
			for (auto j = i; j < it->node_ids.end(); j++)
			{
				if (i == j) continue;
				m_map.addEdge(*i, *j, Edge(it->id, it->length, it->type));
				//m_map.addEdge(*j, *i, Edge(it->id, it->length, it->type));
#ifdef _DEBUG
					fprintf(stdout, "%d %s\n", ++numEdges, "<=======================the number of edges"); // the number of edges
#endif
			}
		}
	}

	return true;
}

bool MapManager::downloadPath(double start_lat, double start_lon, double goal_lat, double goal_lon, int num_paths)
{
	const std::string url_head = "http://129.254.87.96:20005/"; // routing server (paths)
	std::string url = url_head + std::to_string(start_lat) + "/" + std::to_string(start_lon) + "/" + std::to_string(goal_lat) + "/" + std::to_string(goal_lon) + "/" + std::to_string(num_paths);


	return query2server(url);
}

bool MapManager::generatePath(double start_lat, double start_lon, double goal_lat, double goal_lon, int num_paths)
{
	m_path.pts.clear();
	m_json = "";

	// by communication
	downloadPath(start_lat, start_lon, goal_lat, goal_lon, num_paths);
	decodeUni();
	const char* json = m_json.c_str();

#ifdef _DEBUG
	fprintf(stdout, "%s\n", json);
#endif
	Document document;
	document.Parse(json);

	assert(document[0].IsObject());
	const Value& features = document[0]["features"];
	assert(features.IsArray());

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		if (i % 2 == 0)
		{
			Node node = Node(properties["id"].GetUint64());
			m_path.pts.push_back(PathElement(&node, nullptr));
		}
		else
		{
			Edge edge = Edge(properties["id"].GetUint64());
			m_path.pts.push_back(PathElement(nullptr, &edge));
		}
	}

	return true;
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
	Document document;
	document.Parse(json);

	assert(document.IsObject());
	const Value& path = document["path"];
	assert(path.IsArray());
	for (SizeType i = 0; i < path.Size(); i++)
	{
		//m_path.pts.push_back(path[i].GetUint64());
		if (i % 2 == 0)
		{
			Node node = Node(path[i].GetUint64());
			m_path.pts.push_back(PathElement(&node, nullptr));
		}
		else
		{
			Edge edge = Edge(path[i].GetUint64());
			m_path.pts.push_back(PathElement(nullptr, &edge));
		}
	}

	return getPath();
}

Path MapManager::getPath()
{
	return m_path;
}

Map& MapManager::getMap(Path path)
{
	//int num = 0;
	//for (std::list<ID>::iterator node_itr = path.m_points.begin(); node_itr != path.m_points.end(); ++node_itr)
	//{
	//	// exclude edge ID
	//	if(num % 2 != 0)
	//	{
	//		num++;
	//		continue;
	//	}

	//	/*getMap().findNode(*node_itr)->data.lat;
	//	getMap().findNode(*node_itr)->data.lon;*/

	//	num++;
	//}
	

	double lat = 36.38;
	double lon = 127.373;
	double r = 1000.0;


	load(lat, lon, r);

	return getMap();
}

Map& MapManager::getMap()
{
	return m_map;
}

bool MapManager::downloadPOI(double lat, double lon, double radius)
{
	const std::string url_head = "http://129.254.87.96:21502/wgs/";
	std::string url = url_head + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(ID node_id, double radius)
{
	const std::string url_head = "http://129.254.87.96:21502/node/";
	std::string url = url_head + std::to_string(node_id) + "/" + std::to_string(radius);

	return query2server(url);
}

bool MapManager::downloadPOI(cv::Point2i tile)
{
	const std::string url_head = "http://129.254.87.96:21502/tile/";
	std::string url = url_head + std::to_string(tile.x) + "/" + std::to_string(tile.y);

	return query2server(url);
}

std::list<POI>& MapManager::getPOI(double lat, double lon, double radius)
{
	m_map.pois.clear();
	m_json = "";

	// by communication
	downloadPOI(lat, lon, radius);
	decodeUni();
	const char* json = m_json.c_str();
#ifdef _DEBUG
	fprintf(stdout, "%s\n", json);
#endif
	Document document;
	document.Parse(json);

	assert(document.IsObject());
	const Value& features = document["features"];
	assert(features.IsArray());

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		POI poi;
		poi.id = properties["id"].GetUint64();
		poi.name = properties["name"].GetString(); //TODO
		poi.floor = properties["floor"].GetInt();
		poi.lat = properties["latitude"].GetDouble();
		poi.lon = properties["longitude"].GetDouble();

		m_map.pois.push_back(poi);
	}

	return getPOI();
}

std::list<POI>& MapManager::getPOI(ID node_id, double radius)
{
	m_map.pois.clear();
	m_json = "";

	// by communication
	downloadPOI(node_id, radius);
	decodeUni();
	const char* json = m_json.c_str();
#ifdef _DEBUG
	fprintf(stdout, "%s\n", json);
#endif
	Document document;
	document.Parse(json);

	assert(document.IsObject());
	const Value& features = document["features"];
	assert(features.IsArray());

	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		POI poi;
		poi.id = properties["id"].GetUint64();
		poi.name = properties["name"].GetString();	//TODO
		poi.floor = properties["floor"].GetInt();
		poi.lat = properties["latitude"].GetDouble();
		poi.lon = properties["longitude"].GetDouble();

		m_map.pois.push_back(poi);
	}

	return getPOI();
}

std::list<POI>& MapManager::getPOI()
{
	return m_map.pois;
}

std::vector<cv::Point2d> MapManager::getPOI(const char* poiname)
{
	std::vector<cv::Point2d> points;

	m_json = "";

	// by communication
//	downloadPOI(lat, lon, radius); // 1000.0);
//	decodeUni();
//	const char* json = m_json.c_str();
//#ifdef _DEBUG
//	fprintf(stdout, "%s\n", json);
//#endif
//	Document document;
//	document.Parse(json);
//
//	assert(document.IsObject());
//	const Value& features = document["features"];
//	assert(features.IsArray());
//
//	std::vector<EdgeTemp> temp_edge;
//	for (SizeType i = 0; i < features.Size(); i++)
//	{
//		const Value& feature = features[i];
//		assert(feature.IsObject());
//		assert(feature.HasMember("properties"));
//		const Value& properties = feature["properties"];
//		assert(properties.IsObject());
//		std::string name = properties["name"].GetString();
//		if (name == "edge") //continue; //TODO
//		{
//			EdgeTemp edgeinfo;
//			edgeinfo.id = properties["id"].GetUint64();
//			edgeinfo.type = properties["type"].GetInt();
//			edgeinfo.length = properties["length"].GetDouble();
//			temp_edge.push_back(edgeinfo);
//		}
//	}
//
	cv::Point2d point(37.0, 127.0);
	points.push_back(point);
	return points;
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

} // End of 'dg'
