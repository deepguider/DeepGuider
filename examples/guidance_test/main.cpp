#include "dg_core.hpp"
#include "dg_localizer.hpp"
#include "dg_map_manager.hpp"
#include "../unit_test/vvs.h"
#include "guidance.hpp"

//#define USE_PATHFILES 1


std::vector<dg::Guidance::PathInfo> dg::Guidance::getPathExample()
{
	std::vector<PathInfo> path =
	{
		PathInfo(NODE, 559542564800125, POI, 0),	//Start: ETRI gate
		PathInfo(EDGE, 0, SIDEWALK),
		PathInfo(NODE,559542564800923, CORNER, 0),	//ETRI 13B entrance - west
		PathInfo(EDGE, 1, CROSSWALKWOLIGHT),
		PathInfo(NODE,559542564800922, CORNER, 0),	//ETRI 13B entrance - east
		PathInfo(EDGE, 2, SIDEWALK),
		PathInfo(NODE,559552564800175, CORNER, 0),	//Dormitory entrance - west
		PathInfo(EDGE, 3, CROSSWALKWOLIGHT),
		PathInfo(NODE,559552564800052, CORNER, 0),	//Dormitory entrance - east
		PathInfo(EDGE, 4, SIDEWALK),
		PathInfo(NODE,559552564800732, CORNER, 0),	//Kindergarden entrance - west
		PathInfo(EDGE, 5, CROSSWALKWOLIGHT),
		PathInfo(NODE,559552564800733, CORNER, 0),	//Kindergarden entrance - east
		PathInfo(EDGE, 6, SIDEWALK),
		PathInfo(NODE,559562564800752, CORNER, 0),	//SK View entrance - west
		PathInfo(EDGE, 7, CROSSROAD),
		PathInfo(NODE,559562564801071, CORNER, 0),	//SK View entrance - east
		PathInfo(EDGE, 8, SIDEWALK),
		PathInfo(NODE,559562564800771, CORNER, 0),	//Cafe Cocomo entrance - west
		PathInfo(EDGE, 9, CROSSROAD),
		PathInfo(NODE,559562564800834, CORNER, 0),	//Cafe Cocomo entrance - east
		PathInfo(EDGE, 10, SIDEWALK),
		PathInfo(NODE,559562564800828, CORNER, 0),	//Flower entrance - west
		PathInfo(EDGE, 11, CROSSWALKWOLIGHT),
		PathInfo(NODE,559562564800202, CORNER, 0),	//Flower entrance - east
		PathInfo(EDGE, 12, SIDEWALK),
		PathInfo(NODE,559562564800194, CORNER, 45),	//Police office
		PathInfo(EDGE, 13, CROSSWALKWOLIGHT),
		PathInfo(NODE,559562564801444, ISLAND, -45),	//Crosswalk island
		PathInfo(EDGE, 14, CROSSWALKWLIGHT),
		PathInfo(NODE,559562564801442, CORNER, 90),	//Doryong Realestate
		PathInfo(EDGE, 15, CROSSWALKWLIGHT),
		PathInfo(NODE,559562564801032, CORNER, 0),	//GS25
		PathInfo(EDGE, 16, SIDEWALK),
		PathInfo(NODE,559562564700438, POI, -90)	//Goal: Paris Baguette
	};

	return path;
}

std::vector<dg::TopometricPose> dg::Guidance::getPositionExample()
{
	//distance of TopometricPose refers to percentage
	//edge data is not included
	std::vector<dg::TopometricPose> Loc =
	{
		dg::TopometricPose(559542564800125, 0, 0.5),	//ETRI gate
		dg::TopometricPose(559542564800125, 0, 0.9),	//ETRI gate
		dg::TopometricPose(559542564800923, 1, 0),	//ETRI 13B entrance - west
		dg::TopometricPose(559542564800923, 1, 0.5),	//ETRI 13B entrance - west
		dg::TopometricPose(559542564800923, 1, 0.9),	//ETRI 13B entrance - west
		dg::TopometricPose(559542564800922, 2, 0),	//ETRI 13B entrance - east
		dg::TopometricPose(559542564800922, 2, 0.5),	//ETRI 13B entrance - east
		dg::TopometricPose(559542564800922, 2, 0.9),	//ETRI 13B entrance - east		
		dg::TopometricPose(559552564800175, 3, 0),	//Dormitory entrance - west	
		dg::TopometricPose(559552564800175, 3, 0.5),	//Dormitory entrance - west
		dg::TopometricPose(559552564800175, 3, 0.9),	//Dormitory entrance - west		
		dg::TopometricPose(559552564800052, 4, 0),	//Dormitory entrance - east
		dg::TopometricPose(559552564800052, 4, 0.5),	//Dormitory entrance - east
		dg::TopometricPose(559552564800052, 4, 0.9),	//Dormitory entrance - east	
		dg::TopometricPose(559552564800732, 5, 0),	//Kindergarden entrance - west	
		dg::TopometricPose(559552564800732, 5, 0.5),	//Kindergarden entrance - west	
		dg::TopometricPose(559552564800732, 5, 0.9),	//Kindergarden entrance - west		
		dg::TopometricPose(559552564800733, 6, 0),	//Kindergarden entrance - east	
		dg::TopometricPose(559552564800733, 6, 0.5),	//Kindergarden entrance - east	
		dg::TopometricPose(559552564800733, 6, 0.9),	//Kindergarden entrance - east		
		dg::TopometricPose(559562564800752, 7, 0),	//SK View entrance - west
		dg::TopometricPose(559562564800752, 7, 0.5),	//SK View entrance - west
		dg::TopometricPose(559562564800752, 7, 0.9),	//SK View entrance - west		
		dg::TopometricPose(559562564801071, 8, 0),	//SK View entrance - east
		dg::TopometricPose(559562564801071, 8, 0.5),	//SK View entrance - east	
		dg::TopometricPose(559562564801071, 8, 0.9),	//SK View entrance - east		
		dg::TopometricPose(559562564800771, 9, 0),	//Cafe Cocomo entrance - west
		dg::TopometricPose(559562564800771, 9, 0.5),	//Cafe Cocomo entrance - west
		dg::TopometricPose(559562564800771, 9, 0.9),	//Cafe Cocomo entrance - west	
		dg::TopometricPose(559562564800834, 10, 0),	//Cafe Cocomo entrance - east
		dg::TopometricPose(559562564800834, 10, 0.5),	//Cafe Cocomo entrance - east
		dg::TopometricPose(559562564800834, 10, 0.9),	//Cafe Cocomo entrance - east	
		dg::TopometricPose(559562564800828, 11, 0),	//Flower entrance - west
		dg::TopometricPose(559562564800828, 11, 0.5),	//Flower entrance - west	
		dg::TopometricPose(559562564800828, 11, 0.9),	//Flower entrance - west		
		dg::TopometricPose(559562564800202, 12, 0),	//Flower entrance - east
		dg::TopometricPose(559562564800202, 12, 0.5),	//Flower entrance - east	
		dg::TopometricPose(559562564800202, 12, 0.9),	//Flower entrance - east		
		dg::TopometricPose(559562564800194, 13, 0),	//Police office		
		dg::TopometricPose(559562564800194, 13, 0.5),	//Police office		
		dg::TopometricPose(559562564800194, 13, 0.9),	//Police office
		dg::TopometricPose(559562564801444, 14, 0),	//Crosswalk island
		dg::TopometricPose(559562564801444, 14, 0.5),	//Crosswalk island
		dg::TopometricPose(559562564801444, 14, 0.9),	//Crosswalk island	
		dg::TopometricPose(559562564801442, 15, 0),	//Doryong Realestate	
		dg::TopometricPose(559562564801442, 15, 0.5),	//Doryong Realestate	
		dg::TopometricPose(559562564801442, 15, 0.9),	//Doryong Realestate		
		dg::TopometricPose(559562564801032, 16, 0),	//GS25
		dg::TopometricPose(559562564801032, 16, 0.9),	//GS25
		dg::TopometricPose(559562564700438, 16, 0)	//Goal: Paris Baguette
	};

	return Loc;
}

int main()
{
/* //Use this when there is no example.
#if USE_PATHFILES
	//Load path from file
	guider.loadPathFiles("PathNodeIds.txt", guider.m_pathnodeids);
	guider.loadPathFiles("PathEdgeIds.txt", guider.m_pathedgeids);
	
#else
	// Load the map
	dg::MapManager manager;
	manager.load(36.384063, 127.374733, 650.0);
	guider.m_pathNodeIds = manager.getPath("test_simple_Path.json").m_points; //extract ids of nodes
	guider.m_map = manager.getMap();

	//validate path ids. 
	guider.validatePathNodeIds();
	
	//Save file of path's node and edge IDs 
	guider.savePathNodeEdgeIds();
#endif
*/
	dg::Guidance guider;
	dg::SimpleMetricLocalizer localizer;	

	//Load example
	guider.m_path = guider.getPathExample();
	std::vector<dg::TopometricPose> Loc = guider.getPositionExample();

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
/* Use this for simulator.

dg::SimpleRoadMap getExampleMap()
{
// An example road map ('+' represents direction of edges)
// 2 --+ 3 +-+ 5 +-- 6
// +     |     +     |
// |     +     |     +
// 1 +-- 4     7 +-- 8
// example path: 1->2->3->5->7->8

dg::SimpleRoadMap map;
map.addNode(dg::Point2ID(1, 0, 0)); // ID, x, y
map.addNode(dg::Point2ID(2, 0, 1));
map.addNode(dg::Point2ID(3, 1, 1));
map.addNode(dg::Point2ID(4, 1, 0));
map.addNode(dg::Point2ID(5, 2, 1));
map.addNode(dg::Point2ID(6, 3, 1));
map.addNode(dg::Point2ID(7, 2, 0));
map.addNode(dg::Point2ID(8, 3, 0));
map.addEdge(dg::Point2ID(1), dg::Point2ID(2)); // Node#1 -> Node#2
map.addEdge(dg::Point2ID(2), dg::Point2ID(3));
map.addEdge(dg::Point2ID(3), dg::Point2ID(4));
map.addEdge(dg::Point2ID(4), dg::Point2ID(1));
map.addRoad(dg::Point2ID(3), dg::Point2ID(5)); // Add a bi-directional edge
map.addEdge(dg::Point2ID(5), dg::Point2ID(6));
map.addEdge(dg::Point2ID(6), dg::Point2ID(8));
map.addEdge(dg::Point2ID(5), dg::Point2ID(7));
map.addEdge(dg::Point2ID(7), dg::Point2ID(8));
return map;
}

std::vector<std::pair<std::string, cv::Vec3d>> getExampleDataset()
{
std::vector<std::pair<std::string, cv::Vec3d>> dataset =
{
std::make_pair("Pose",      cv::Vec3d(0, 0, cx::cvtDeg2Rad(95))),

std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.1)),  // RefNode#, Edge#(RefNode# & NextNode#, Each 4-Digit without left-most 0's), Distance from RefNode
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.2)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.3)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.4)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.5)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.6)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.7)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.8)),
std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.9)),
//      std::make_pair("LocClue",   cv::Vec3d(2, -1, CV_PI)),

std::make_pair("Pose",      cv::Vec3d(0, 1, cx::cvtDeg2Rad(-5))),

std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.1)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.2)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.3)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.4)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.5)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.6)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.7)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.8)),
std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.9)),
//      std::make_pair("LocClue",   cv::Vec3d(3, -1, CV_PI)),

std::make_pair("Pose",      cv::Vec3d(1, 1, cx::cvtDeg2Rad(0))),

std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.1)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.2)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.3)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.4)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.5)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.6)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.7)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.8)),
std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(5, -1, CV_PI)),

std::make_pair("Pose",      cv::Vec3d(2, 1, cx::cvtDeg2Rad(-85))),

std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.1)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.2)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.3)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.4)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.5)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.6)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.7)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.8)),
std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(7, -1, CV_PI)),

std::make_pair("Pose",      cv::Vec3d(2, 0, cx::cvtDeg2Rad(-5))),

std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.1)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.2)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.3)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.4)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.5)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.6)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.7)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.8)),
std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(8, -1, CV_PI)),

std::make_pair("Pose",      cv::Vec3d(3, 0, cx::cvtDeg2Rad(90))),

};
return dataset;
}

std::vector<std::tuple<std::string, cv::Vec3d, std::string>> getExampleDataset()
{
std::vector<std::tuple<std::string, cv::Vec3d, std::string>> dataset =
{
std::make_tuple("Pose",      cv::Vec3d(0, 0, cx::cvtDeg2Rad(95)), "Ready to start"),

// cv::Vec3d => RefNode#, Edge#(RefNode# & NextNode#, Each 4-Digit without left-most 0's), Distance from RefNode
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.1), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.2), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.3), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.4), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.5), "Go straight to next node. You are in the middle point."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.6), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.7), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.8), "Go straight to next node. You almost arrived."),
std::make_tuple("Odometry",  cv::Vec3d(1, 10002, 0.9), "Go straight to next node and turn right after that."),

//      std::make_tuple("LocClue",   cv::Vec3d(2, -1, CV_PI), "You arrived node #2. Turn right."),

std::make_tuple("Pose",      cv::Vec3d(0, 1, cx::cvtDeg2Rad(-5)), "You arrived node #2. Turn right."),

std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.1), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.2), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.3), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.4), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.5), "Go straight to next node. You are in the middle point."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.6), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.7), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.8), "Go straight to next node. You almost arrived."),
std::make_tuple("Odometry",  cv::Vec3d(2, 20003, 0.9), "Go straight to next node and cross a crosswalk after that."),

//      std::make_tuple("LocClue",   cv::Vec3d(3, -1, CV_PI), "You arrived node #3. Cross a crosswalk."),

std::make_tuple("Pose",      cv::Vec3d(1, 1, cx::cvtDeg2Rad(0)), "You arrived node #3. Cross a crosswalk."),

std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.1), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.2), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.3), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.4), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.5), "Be careful to cross a crosswalk. You are in the middle point."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.6), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.7), "Be careful to cross a crosswalk."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.8), "Be careful to cross a crosswalk. You almost done."),
std::make_tuple("Odometry",  cv::Vec3d(3, 30005, 0.9), "Be careful to cross a crosswalk and turn right after that."),

//		std::make_tuple("LocClue",   cv::Vec3d(5, -1, CV_PI), "You arrived node #5. Turn right."),

std::make_tuple("Pose",      cv::Vec3d(2, 1, cx::cvtDeg2Rad(-85)), "You arrived node #5. Turn right."),

std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.1), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.2), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.3), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.4), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.5), "Go straight to next node. You are in the middle point."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.6), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.7), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.8), "Go straight to next node. You almost arrived."),
std::make_tuple("Odometry",  cv::Vec3d(5, 50007, 0.9), "Go straight to next node and turn left after that."),

//		std::make_tuple("LocClue",   cv::Vec3d(7, -1, CV_PI), "You arrived node #7. Turn left."),

std::make_tuple("Pose",      cv::Vec3d(2, 0, cx::cvtDeg2Rad(-5)), "You arrived node #7. Turn left."),

std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.1), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.2), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.3), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.4), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.5), "Go straight to next node. You are in the middle point."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.6), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.7), "Go straight to next node."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.8), "Go straight to next node. You almost arrived."),
std::make_tuple("Odometry",  cv::Vec3d(7, 70008, 0.9), "Go straight to next node. The goal is nearby."),

//		std::make_tuple("LocClue",   cv::Vec3d(8, -1, CV_PI), "Now you arrived at the goal point."),

std::make_tuple("Pose",      cv::Vec3d(3, 0, cx::cvtDeg2Rad(90)), "Now you arrived at the goal point."),

};
return dataset;
}

std::vector<dg::NodeInfo> getExamplePath()
{
//Path generation should be modified.
//It does not contain x,y coordinates.(2019-09-20 JSH)
std::vector<dg::NodeInfo> path =
{
dg::NodeInfo (1, 0, 0, 0, 1),
dg::NodeInfo (2, 0, 1, 0, 1),
dg::NodeInfo (3, 1, 1, 1, 1),
dg::NodeInfo (5, 2, 1, 1, 1),
dg::NodeInfo (7, 2, 0, 0, 1),
dg::NodeInfo (8, 3, 0, 0, 1)
};

return path;
}

int main()
{
    // Load a map
    dg::SimpleRoadMap map = getExampleMap();
    dg::SimpleMetricLocalizer localizer;
    if (!localizer.loadMap(map)) return -1;

	//added by seohyun
	std::vector<dg::NodeInfo> path = getExamplePath();


    // Prepare visualization
	dg::SimpleRoadPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 200)) return -1;
    if (!painter.setParamValue("node_font_scale", 2 * 0.5)) return -1;
    dg::CanvasInfo map_info = painter.getCanvasInfo(map);
    cv::Mat map_image;
    if (!painter.drawMap(map_image, map)) return -1;

    // Run localization
    auto dataset = getExampleDataset();
	cv::Vec3d preDist = (0, 0);
	cv::Vec3d curDist = (0, 0);

    for (size_t t = 0; t < dataset.size(); t++)
    {
        const cv::Vec3d& d = std::get<1>(dataset[t]);
		
//      if (dataset[t].first == "Pose")     localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t);
//		if (dataset[t].first == "Odometry") localizer.applyOdometry(dg::Polar2(d[0], d[1]), t);
//      if (dataset[t].first == "LocClue")  localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), t);

		if (std::get<0>(dataset[t]) == "Pose")
		{
			preDist = (0, 0);
			curDist = (0, 0);
			localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t);
		}
		if (std::get<0>(dataset[t]) == "Odometry")
		{
			// d[0]=RefNode#, d[1]=Edge#(RefNode# & NextNode#), d[2]=Distance from RefNode
			curDist[0] = d[2] - preDist[0];
			preDist[0] = d[2];
			localizer.applyOdometry(dg::Polar2(curDist[0], curDist[1]), t);
		}

        cv::Mat image = map_image.clone();
		
        dg::Pose2 pose = localizer.getPose();
        painter.drawNode(image, map_info, dg::Point2ID(0, pose.x, pose.y), 0.1, 0, cx::COLOR_MAGENTA);

		cv::putText(image, std::get<2>(dataset[t]), cv::Point(40, 335), 1, 1.3, cv::Scalar(0, 0, 255));
		
		cv::imshow("Simple Test", image);

        int key = cv::waitKeyEx();
        if (key == cx::KEY_ESC) return -1;
    }
    return 0;
}
*/