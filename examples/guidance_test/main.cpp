#include "dg_core.hpp"
#include "dg_localizer.hpp"
#include "dg_map_manager.hpp"
#include "guidance\guidance.hpp"

int main()
{
	dg::Guidance guider;

	//Load example
	guider.loadPathFiles("Path_ETRIFrontgateToParisBaguette.txt", guider.m_path);
	std::vector<dg::TopometricPose> Loc;
	guider.loadLocFiles("Loc_ETRIFrontgateToParisBaguette.txt", Loc);

	//generate guide
	guider.generateGuide();	
	
	//Initial move
	dg::Guidance::Guide initG(guider.m_guide[0]);
	dg::Guidance::Action InitA(dg::Guidance::GO_FORWARD, 0);
	std::vector<dg::Guidance::InstantGuide> curGuide;
	curGuide.push_back(dg::Guidance::InstantGuide(initG, InitA));


	dg::TopometricPose curPose;
	dg::Guidance::Status curStatus;
	//current locClue && current path
	for (size_t i = 0; i < Loc.size(); i++)
	{
		curPose = Loc[i];
		curStatus = guider.checkStatus(curPose);
		curGuide = guider.provideNormalGuide(curGuide, curStatus);		
		fprintf(stdout, "Step: %d\n", i);
		fprintf(stdout, "[Pose] Node: %" PRIu64 ", Base edge: %" PRIu64 ", Dist: %.1f\n", curPose.node_id, curPose.edge_idx, curPose.dist);
		fprintf(stdout, "[Status] %d\n", curStatus);
		for (size_t j = 0; j < curGuide.size(); j++)
		{
			fprintf(stdout, "[Guide] Towards: %" PRIu64 ", NodeType: %d, Mode(EdgeType): %d, Motion: %d, Direction: %d\n", curGuide[j].guide.nodeid, curGuide[j].guide.type, curGuide[j].guide.mode, curGuide[j].action.move, curGuide[j].action.direction);
		}		
		fprintf(stdout, "\n");
	}
	return 0;
}