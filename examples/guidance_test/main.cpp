#include "dg_core.hpp"
#include "dg_localizer.hpp"
#include "dg_map_manager.hpp"
#include "guidance/guidance.hpp"

#define USE_EXAMPLE_PATHFILE 0

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
			Node* node = map.findNode(id);
			pathtemp.node = node;
		}
		else // edge
		{
			Edge* edge = map.findEdge(id);
			pathtemp.edge = edge;
			pathvec.push_back(pathtemp);
		}
		count++;

	}

	pathtemp.edge = nullptr;
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

int main()
{	   	 
	MapManager map_manager;
	map_manager.setIP("localhost");
	
	// Get the path & map
	bool ok;
	dg::Map map;
	dg::Path path;
	ok = map_manager.getMap(559542564800095, 700, map);
	//ok = map_manager.getMap(36.382967999999998, 127.37138150000001, 700, map);
	ok = map_manager.getPath(36.381873, 127.36803, 36.384063, 127.374733, path);
	
	printf("Paths\n");
	for (size_t i = 0; i < path.pts.size()-1; i++)
	{
		//Node* curnode = map.findNode(path.pts[i].node->id);
		//Edge* curedge = map.findEdge(path.pts[i].edge->id);
		//printf("[%zd]: Node-%zu, Edge-%zu\n", i, curnode->id, curedge->id);
		printf("[%zd]: Node-%zu, Edge-%zu\n", i, path.pts[i].node->id, path.pts[i].edge->id);
	}
	printf("[%zd]: Node-%zu\n", path.pts.size() - 1, path.pts[path.pts.size() - 1].node->id);

	// generate path to the destination
	// load pre-defined path for the test
	PathElement start_node = path.pts.front();
	PathElement dest_node = path.pts.back();
	printf("\tSample Path generated! start=%zu, dest=%zu\n", start_node.node->id, dest_node.node->id);

	   
#if USE_EXAMPLE_PATHFILE
	//Load example
	guider.loadPathFiles("Path1.txt", guider.m_path);
	std::vector<TopometricPose> Loc;
	guider.loadLocFiles("Loc_Path1Test.txt", Loc);

#else

	GuidanceManager guider;
	guider.setPathNMap(path, map);
	guider.initializeGuides();

	std::vector<TopometricPose> Loc;
	loadLocFiles("Loc_Path1Test.txt", Loc);

	//Initial move
	GuidanceManager::Guidance curGuide;
	TopometricPose curPose;
	GuidanceManager::MoveStatus curStatus;

	//current locClue && current path
	for (size_t i = 0; i < Loc.size(); i++)
	{
		fprintf(stdout, "Step: %zd\n", i);

		//print current TopometricPose
		curPose = Loc[i];
		fprintf(stdout, "[Pose] Node: %zu, Dist: %.1f\n", curPose.node_id, curPose.dist);

		curStatus = guider.applyPose(curPose);

		//print status
		std::string str_status;
		switch (curStatus)
		{
		case GuidanceManager::MoveStatus::ON_EDGE:
			str_status = "ON_EDGE";
			break;
		case GuidanceManager::MoveStatus::APPROACHING_NODE:
			str_status = "APPROACHING_NODE";
			break;
		case GuidanceManager::MoveStatus::ON_NODE:
			str_status = "ON_NODE";
			break;
		default:
			break;
		}
		fprintf(stdout, "[Status] %s\n", str_status.c_str());

		curGuide = guider.getGuidance(curStatus);
		//curGuide = guider.getGuidance(curPose);

		//print guidance message
		fprintf(stdout, "%s\n", curGuide.msg.c_str());
	}
	fprintf(stdout, "\n Press any key to finish.\n");

#endif
	
	getchar();
	
	return 0;
}
