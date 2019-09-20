#include "Guidance.hpp"

using namespace dg;

#define                                    PI                                                      3.141592
#define                                    RADIAN                                        ( PI / 180.0 )
#define                                    DEGREE                                       ( 180.0 / PI )
#define                                    RAD2DEG( Rad  )                    ( Rad * DEGREE )
#define                                    DEG2RAD( Degree )            ( Degree * RADIAN )

double GetAngleBetweenTwoVectors(double dVec1X, double dVec1Y, double dVec2X, double dVec2Y)
{
	double dAngle1 = RAD2DEG(atan2(dVec1X, dVec1Y));
	double dAngle2 = RAD2DEG(atan2(dVec2X, dVec2Y));

	double dDiffAngles = dAngle1 - dAngle2;

	if (dDiffAngles < 0)
		dDiffAngles = 360 + dDiffAngles;

	return dDiffAngles;
}


std::vector<Guidance::ActionType<Guidance::Motion, int>> dg::Guidance::generateGuide(std::vector<NodeInfo>& path)
{
	std::vector<Guidance::ActionType <Guidance::Motion, int>> guidance;

	Guidance::ActionType <Guidance::Motion, int> initAct(Guidance::FORWARD, 12);
	guidance.push_back(initAct);

	//Path decomposition is proceeded below.
	//first guide

	for (size_t i = 1; i < path.size(); i++)
	{
		//double angle = GetAngleBetweenTwoVectors(path[i].lon, path[i].lat, path[i + 1].lon, path[i + 1].lat);		
		//vector(path[i+1] - path[i]) X vector(path[i] - path[i-1])
	}

	return guidance;
}
