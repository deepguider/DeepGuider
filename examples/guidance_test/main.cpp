#include "dg_core.hpp"
#include "dg_localizer.hpp"
#include "dg_map_manager.hpp"
#include "guidance/guidance.hpp"

using namespace dg;

Path getPath(const char* filename, Map& map)
{
	std::vector<PathElement> pathvec;

	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line;
	char data[1024];
	char* token;

	PathElement pathtemp;
	int count = 0;
	while (std::getline(is, line))
	{
		strcpy(data, line.c_str());
		token = strtok(data, ",");
		ID id = (ID)atoll(token);

		if (count%2 == 0)	// node
		{
			//Node* node = map.findNode(id);
			pathtemp.node_id = id;
		}
		else // edge
		{
			//Edge* edge = map.findEdge(id);
			pathtemp.edge_id = id;
			pathvec.push_back(pathtemp);
		}
		count++;

	}

	pathtemp.edge_id = 0;
	pathvec.push_back(pathtemp);
	
	Path path;
	path.pts = pathvec;

	return path;

}

bool loadLocFiles(const char* filename, std::vector<TopometricPose>& destination)
{
	destination.clear();

	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line;
	char data[1024];
	char* token;
	while (std::getline(is, line))
	{
		strcpy(data, line.c_str());
		token = strtok(data, "(,)");
		ID nodeid = (ID)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		int edgeidx = (int)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		double dist = strtod(token, NULL);

		TopometricPose pose = TopometricPose(nodeid, edgeidx, dist);
		destination.push_back(pose);
	}

	is.close();
	return true;

GUIDANCE_LOADPATH_FAIL:
	destination.clear();
	is.close();
	return false;

}

bool loadLocConfFiles(const char* filename, std::vector<TopometricPose>& destination, std::vector<double>& confs)
{
	destination.clear();
	confs.clear();

	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line;
	char data[1024];
	char* token;
	while (std::getline(is, line))
	{
		strcpy(data, line.c_str());
		token = strtok(data, "(,)");
		ID nodeid = (ID)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		int edgeidx = (int)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		double dist = strtod(token, NULL);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		double conf = strtod(token, NULL);

		TopometricPose pose = TopometricPose(nodeid, edgeidx, dist);
		destination.push_back(pose);
		confs.push_back(conf);
	}

	is.close();
	return true;

GUIDANCE_LOADPATH_FAIL:
	destination.clear();
	confs.clear();
	is.close();
	return false;

}


std::vector<std::pair<double, dg::LatLon>> getExampleGPSData(const char* csv_file = "data/191115_ETRI_asen_fix.csv")
{
	cx::CSVReader csv;
	cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header

	std::vector<std::pair<double, dg::LatLon>> data;
	for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
	{
		double timestamp = row->at(0) + 1e-9 * row->at(1);
		dg::LatLon ll(row->at(2), row->at(3));
		data.push_back(std::make_pair(timestamp, ll));
	}
	return data;
}


int main()
{	   	 

	MapManager map_manager;
	map_manager.setIP("localhost");
	
	// Get the path & map
	bool ok;
	dg::Path path, newPath;
//	ok = map_manager.getPath(36.381873, 127.36803, 36.384063, 127.374733, path);
	ok = map_manager.getPath(36.38205717, 127.3676462, 36.37944417, 127.3788568, path);
	dg::Map map = map_manager.getMap();
	printf("Original Paths\n");
	for (size_t i = 0; i < path.pts.size(); i++)
	{
		printf("[%zd]: ", i);
		Node* curnode = map.findNode(path.pts[i].node_id);
		if (curnode != nullptr)
			printf("Node-%zu, ", curnode->id);
		else
			printf("No node(%zu) is found on map!, ", path.pts[i].node_id);
		Edge* curedge = map.findEdge(path.pts[i].edge_id);
		if (curedge != nullptr)
			printf(" Edge-%zu\n", curedge->id);
		else
			printf("No edge(%zu) is found on map!\n", path.pts[i].edge_id);
		//printf("[%zd]: Node-%zu, Edge-%zu\n", i, curnode->id, curedge->id);
		//printf("[%zd]: Node-%zu, Edge-%zu\n", i, path.pts[i].node_id, path.pts[i].edge_id);
	}
	 //for (size_t i = 14; i < 26; i++)
	 //{
	 //	Node* curnode = map.findNode(path.pts[i].node_id);
	 //	Edge* curedge = map.findEdge(path.pts[i].edge_id);
	 //	newPath.pts.push_back(PathElement(curnode->id, curedge->id));
	 //	printf("[%zd]: Node-%zu, Edge-%zu\n", i, curnode->id, curedge->id);
	 //	//printf("[%zd]: Node-%zu, Edge-%zu\n", i, path.pts[i].node_id, path.pts[i].edge_id);
	 //}
	 //Node* curnode = map.findNode(path.pts[26].node_id);
	 //newPath.pts.push_back(PathElement(curnode->id, 0));
	 //path = newPath;

	// PathElement start_node = newPath.pts.front();
	// PathElement dest_node = newPath.pts.back();
	//printf("[%zd]: Node-%zu\n", path.pts.size() - 1, path.pts[path.pts.size() - 1].node_id);
	
	//auto gps_data = getExampleGPSData("data/191115_ETRI_asen_fix.csv");
	//printf("\tSample gps data loaded!\n");

	// generate path to the destination
	// load pre-defined path for the test
	PathElement start_node = path.pts.front();
	PathElement dest_node = path.pts.back();
	printf("\tSample Path generated! start=%zu, dest=%zu\n", start_node.node_id, dest_node.node_id);


	GuidanceManager guider;
	if (!guider.setPathNMap(path, map))
	{
		getchar();
		return 0;
	}
	guider.initializeGuides();
	
	std::vector<TopometricPose> Loc;
	std::vector<double> Confs;
	//loadLocFiles("Loc_Path1Test.txt", Loc);
	//loadLocConfFiles("Loc_Path3Test.txt", Loc, Confs);
	loadLocConfFiles("Loc_Path1Test.txt", Loc, Confs);

	std::string str_status[4] = { "ON_NODE", "ON_EDGE", "APPROACHING_NODE"
		, "ARRIVED" };


	//Initial move
	GuidanceManager::Guidance curGuide;
	GuidanceManager::GuideStatus curGStatus;
	TopometricPose curPose;
	double curConf;
	GuidanceManager::MoveStatus curStatus;

	//current locClue && current path
	for (size_t i = 0; i < Loc.size(); i++)
	{
		fprintf(stdout, "Step: %zd\n", i);

		//print current TopometricPose
		curPose = Loc[i];
		fprintf(stdout, "[Pose] Node: %zu, Dist: %.1f\n", curPose.node_id, curPose.dist);

		curConf = Confs[i];
		curGStatus = guider.getGuidanceStatus(curPose, curConf);
		curGuide = guider.getGuidance(curPose, curGStatus);

		//save latest GPS
		Node* node = map.findNode(curPose.node_id);
		if (node != nullptr)
		{
			guider.applyPoseGPS(LatLon(node->lat, node->lon));
		}

		//print status
		fprintf(stdout, "[Status] %s\n", str_status[(int)curGuide.moving_status].c_str());
		curGuide = guider.getGuidance(curPose);

		//print guidance message
		fprintf(stdout, "%s\n", curGuide.msg.c_str());
	}
	fprintf(stdout, "\n Press any key to finish.\n");
		
	getchar();
	
	return 0;
}
