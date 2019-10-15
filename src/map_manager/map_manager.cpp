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

void MapManager::downloadMap(cv::Point2i tile)
{
	const std::string url_head = "https://path.to.topological.map.server/";
	std::string url = url_head + std::to_string(tile.x) + "/" + std::to_string(tile.y) + "/";

}

bool MapManager::load(double lon, double lat, int z)
{
    m_map.removeAll();

	downloadMap(lonlat2xy(lon, lat, z));

	// Convert JSON document to string
	const char* filename = "test_simple_map.json";
	auto is = std::ifstream(filename, std::ofstream::in);
	if (!is.is_open())
	{
		std::cerr << "Could not open file for reading!\n";
		return EXIT_FAILURE;
	}
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
	}
		
	return true;
}

size_t write_callback(void* ptr, size_t size, size_t count, void* stream)
{
	((std::string*)stream)->append((char*)ptr, 0, size * count);
	return size * count;
}

bool MapManager::generatePath()
{
	SetConsoleOutputCP(65001);

	std::string client_id = "X-NCP-APIGW-API-KEY-ID:h2weu1vyc4";
	std::string	client_secret = "X-NCP-APIGW-API-KEY:xYSNKDADst7RfmiFnMAyi1EkTcWqusBV1oHwVmax";
	std::string url{ "https://naveropenapi.apigw.ntruss.com/map-direction/v1/driving?start=127.31788462835466,36.37407414112156,start_pos&goal=127.3190043087742,36.37253204490351,end_pos&option=trafast" };

	curl_global_init(CURL_GLOBAL_ALL);
	CURL *curl = curl_easy_init();
	CURLcode res;
	struct curl_slist* headers = NULL;
	
	if (curl) 
	{
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		headers = curl_slist_append(headers, client_id.c_str());
		headers = curl_slist_append(headers, client_secret.c_str());
		curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers); /* pass our list of custom made headers */

		std::string response;
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

		// Perform the request, res will get the return code.
		res = curl_easy_perform(curl);

		// Always cleanup.
		curl_slist_free_all(headers);
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
			fprintf(stdout, "%s\n", response.c_str());
		}
	}

	return true;
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
