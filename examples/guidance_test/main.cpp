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
		fprintf(stdout, "Step: %d\n", i);

		curPose = Loc[i];
		fprintf(stdout, "[Pose] Node: %" PRIu64 ", Base edge: %" PRIu64 ", Dist: %.1f\n", curPose.node_id, curPose.edge_idx, curPose.dist);

		curStatus = guider.checkStatus(curPose);
		fprintf(stdout, "[Status] %d\n", curStatus);

		curGuide = guider.provideNormalGuide(curGuide, curStatus);	
		for (size_t j = 0; j < curGuide.size(); j++)
		{
			guider.printInstantGuide(curGuide[j]);
		}		
		fprintf(stdout, "\n");
	}
	return 0;
}