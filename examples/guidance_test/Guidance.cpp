#include "Guidance.hpp"

using namespace dg;

std::vector<Guidance::ActionType <Guidance::Motion, int>> generateGuide(std::vector<dg::NodeInfo> path)
{
	std::vector<Guidance::ActionType <Guidance::Motion, int>> guidance;

	Guidance::ActionType <Guidance::Motion, int> initAct(Guidance::FORWARD, 12);
	guidance.push_back(initAct);

	//Path decomposition is proceeded below.
	for (size_t i = 0; i < path.size(); i++)
	{
		
	}

	return guidance;
}
