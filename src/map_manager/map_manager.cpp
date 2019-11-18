#include "map_manager.hpp"

namespace dg
{

	int MapManager::long2tilex(double lon, int z)
	{
		return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
	}

	int MapManager::lat2tiley(double lat, int z)
	{
		double latrad = lat * M_PI / 180.0;
		return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)));
	}

	double MapManager::tilex2long(int x, int z)
	{
		return x / (double)(1 << z) * 360.0 - 180;
	}

	double MapManager::tiley2lat(int y, int z)
	{
		double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
		return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
	}

	cv::Point2i MapManager::lonlat2xy(double lon, double lat, int z)
	{
		return cv::Point2i(long2tilex(lon, z), lat2tiley(lat, z));
	}

	size_t write_callback(void* ptr, size_t size, size_t count, void* stream)
	{
		((std::string*)stream)->append((char*)ptr, 0, size * count);
		return size * count;
	}
	
	bool MapManager::query2server(std::string url)
	{
		SetConsoleOutputCP(65001);

		//std::string client_id = "X-NCP-APIGW-API-KEY-ID:h2weu1vyc4";
		//std::string	client_secret = "X-NCP-APIGW-API-KEY:xYSNKDADst7RfmiFnMAyi1EkTcWqusBV1oHwVmax";
		//std::string url{ "https://naveropenapi.apigw.ntruss.com/map-direction/v1/driving?start=127.31788462835466,36.37407414112156,start_pos&goal=127.3190043087742,36.37253204490351,end_pos&option=trafast" };

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

void MapManager::downloadMap(cv::Point2i tile)
{
	const std::string url_head = "https://path.to.topological.map.server/";
	std::string url = url_head + std::to_string(tile.x) + "/" + std::to_string(tile.y) + "/";

}

bool MapManager::downloadMap(double lat, double lon, double radius)
{
	const std::string url_head = "http://129.254.87.96:21500/wgs/"; // routing server (nodes)
	std::string url = url_head + std::to_string(lat) + "/" + std::to_string(lon) + "/" + std::to_string(radius);
	

	return query2server(url);
}

// unicode-escape decording
std::string to_utf8(uint32_t cp)
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

bool MapManager::load(double lat, double lon, double radius)//(double lon, double lat, int z)
{
    m_map.removeAll();

	// by file
	//downloadMap(lonlat2xy(lon, lat, z));
	//// Convert JSON document to string
	//const char* filename = "test_simple_map.json";
	//auto is = std::ifstream(filename, std::ofstream::in);
	//if (!is.is_open())
	//{
	//	std::cerr << "Could not open file for reading!\n";
	//	return EXIT_FAILURE;
	//}
	//std::string line, text;
	//while (std::getline(is, line))
	//{
	//	text += line + "\n";
	//}
	//const char* json = text.c_str();
	//is.close();

	// by communication
	downloadMap(36.383921, 127.367481, 16.0); // 1000.0);
	
	// unicode-escape decording
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

	const char* json = m_json.c_str();
	fprintf(stdout, "%s\n", json);
	Document document;
	document.Parse(json);

	/*// test_simple_map.json
	assert(document.IsObject());
	const Value& nodes = document["nodes"];
	assert(nodes.IsArray());
	for (SizeType i = 0; i < nodes.Size(); i++)
	{
		const Value& node = nodes[i];
		NodeInfo nodeinfo;
		nodeinfo.id = node["id"].GetUint64();
		nodeinfo.lon = node["longitude"].GetDouble();
		nodeinfo.lat = node["latitude"].GetDouble();
		const Value& streetviews = node["streetviews"];
		for (Value::ConstValueIterator streetview = streetviews.Begin(); streetview != streetviews.End(); ++streetview)
			nodeinfo.sv_ids.push_back(streetview->GetUint64());
		const Value& pois = node["pois"];
		for (Value::ConstValueIterator poi = pois.Begin(); poi != pois.End(); ++poi)
			nodeinfo.pois.push_back(poi->GetString());
		nodeinfo.type = node["node_type"].GetInt();
		nodeinfo.floor = node["floor"].GetInt();

		m_map.addNode(nodeinfo);
	}
	for (SizeType i = 0; i < nodes.Size(); i++)
	{
		const Value& node = nodes[i];
		NodeInfo from_node;
		from_node.id = node["id"].GetUint64();
		const Value& edges = node["edges"];
		EdgeInfo edgeinfo;
		for (SizeType j = 0; j < edges.Size(); j++)
		{
			const Value& edge = edges[j];
			ID tid = edge["tid"].GetUint64();
			edgeinfo.width = edge["width"].GetDouble();
			edgeinfo.length = edge["length"].GetDouble();
			edgeinfo.type = edge["edge_type"].GetInt();

			m_map.addEdge(from_node, NodeInfo(tid), edgeinfo);
		}
	}*/
		
	assert(document.IsObject());
	const Value& features = document["features"];
	assert(features.IsArray());
	for (SizeType i = 0; i < features.Size(); i++)
	{
		const Value& feature = features[i];
		NodeInfo nodeinfo;
		assert(feature.IsObject());
		assert(feature.HasMember("properties"));
		const Value& properties = feature["properties"];
		assert(properties.IsObject());
		std::string name = properties["name"].GetString();
		if (name == "edge") continue; //TODO
		nodeinfo.id = properties["id"].GetUint64();
		nodeinfo.floor = properties["floor"].GetInt();
		nodeinfo.lat = properties["latitude"].GetDouble();
		nodeinfo.lon = properties["longitude"].GetDouble();
		m_map.addNode(nodeinfo);
		EdgeInfo edgeinfo;
		/*const Value& edges = feature["edges"];
		for (SizeType j = 0; j < edges.Size(); j++)
		{
			const Value& edge = edges[j];
			ID tid = edge["tid"].GetUint64();
			edgeinfo.width = edge["width"].GetDouble();
			edgeinfo.length = edge["length"].GetDouble();
			edgeinfo.type = edge["edge_type"].GetInt();

			m_map.addEdge(from_node, NodeInfo(tid), edgeinfo);
		}*/
		const Value& edge_ids = properties["edge_ids"];
		for (Value::ConstValueIterator edge_id = edge_ids.Begin(); edge_id != edge_ids.End(); ++edge_id)
		{
			edgeinfo.width = 0.0; //TODO
			edgeinfo.length = 0.0; //TODO
			edgeinfo.type = 0; //TODO
			m_map.addEdge(nodeinfo.id, NodeInfo(edge_id->GetUint64()), edgeinfo);
		}
	}

	return true;
}

bool MapManager::generatePath()
{
	

	return downloadMap(36.383921, 127.367481, 1000.0);
}

Path MapManager::getPath(const char* filename)
{
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
		m_path.m_points.push_back(path[i].GetUint64());
	}

	return m_path;
}

Map& MapManager::getMap(Path path)
{
	//TODO
	CURL *curl;

	double lon = 128;
	double lat = 37.5;
	int z = 19;

	load(lon, lat, z);

	return getMap();
}

Map& MapManager::getMap()
{
	return m_map;
}

std::vector<cv::Point2d> MapManager::getPOIloc(const char* poiname)
{
	std::vector<cv::Point2d> points;
	for (dg::Map::NodeItr node_itr = m_map.getHeadNode(); node_itr != m_map.getTailNode(); node_itr++)
	{
		for (std::vector<std::string>::iterator it = node_itr->data.pois.begin(); it != node_itr->data.pois.end(); it++)
		{
			if (*(it) == poiname)
			{
				cv::Point2d point(node_itr->data.lon, node_itr->data.lat);
				points.push_back(point);
				break;
			}
		}
	}
	return points;
}

} // End of 'dg'
