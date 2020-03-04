#include "dg_core.hpp"
#include "dg_localizer.hpp"
#include "dg_map_manager.hpp"
#include "guidance/guidance.hpp"
#include "../unit_test/vvs.h"

#define USE_JSH_PATHFILE 0

int main()
{
	   	 
	dg::MapManager m_map_manager;

	// generate path to the destination

	// load pre-defined path for the test
	dg::Path path = m_map_manager.getPath("Path1.json");
	dg::ID start_node = path.m_points.front();
	dg::ID dest_node = path.m_points.back();
	printf("\tSample Path generated! start=%llu, dest=%llu\n", start_node, dest_node);

	// load map along the path
	if (!m_map_manager.load(36.384063, 127.374733, 650.0))
	{
		printf("\tFailed to load sample map!\n");
	}
	printf("\tSample Map loaded!\n");

	// set map to localizer
	dg::Map& map = m_map_manager.getMap();		
	   
#if USE_JSH_PATHFILE
	//Load example
	guider.loadPathFiles("Path_ETRIFrontgateToParisBaguette.txt", guider.m_path);
	std::vector<dg::TopometricPose> Loc;
	guider.loadLocFiles("Loc_ETRIFrontgateToParisBaguette.txt", Loc);

#else

	dg::Guidance guider;
	guider.setPathNMap(path, map);
	guider.initializeGuides();

	std::vector<dg::TopometricPose> Loc;
	guider.loadLocFiles("Loc_Path1Test.txt", Loc);

	//Initial move
	std::vector<dg::Guidance::RobotGuide> curGuide;
	curGuide = guider.getInitGuide();

	dg::TopometricPose curPose;
	dg::Guidance::MoveStatus curStatus;
	//current locClue && current path
	for (size_t i = 0; i < Loc.size(); i++)
	{
		fprintf(stdout, "Step: %zd\n", i);

		//print current TopometricPose
		curPose = Loc[i];
		fprintf(stdout, "[Pose] Node: %llu, Dist: %.1f\n", curPose.node_id, curPose.dist);

		curStatus = guider.applyPose(curPose);

		//print status
		std::string str_status;
		switch (curStatus)
		{
		case dg::Guidance::MoveStatus::ON_EDGE:
			str_status = "ON_EDGE";
			break;
		case dg::Guidance::MoveStatus::APPROACHING_NODE:
			str_status = "APPROACHING_NODE";
			break;
		case dg::Guidance::MoveStatus::ARRIVED_NODE:
			str_status = "ARRIVED_NODE";
			break;
		default:
			break;
		}
		fprintf(stdout, "[Status] %s\n", str_status.c_str());

		curGuide = guider.getNormalGuide(curStatus);

		//print robot guide
		guider.printRobotGuide(curGuide);
		fprintf(stdout, "\n");
	}
	fprintf(stdout, "Arrived!\n");

#endif
	
	getchar();
	
	return 0;
}
