#pragma once
#include "dg_core.hpp"

namespace dg
{

class Guidance
{
public:

	enum NodeType
	{
		POI = 0,	//CONTINOUS ROAD, 180 DEGREE
		CORNER,		//CONTINOUS ROAD, 90 DEGREE
		ISLAND		//DISCONNECTED ROAD
	};

	enum EdgeType
	{
		SIDEWALK = 0,
		CROSSWALKWOLIGHT,
		CROSSWALKWLIGHT,
		CROSSROAD,
		ROAD
	};

	enum PathType
	{
		NODE = 0,
		EDGE
	};
	class PathInfo
	{
	public:
		PathInfo(PathType _component = NODE, ID _id = 0, NodeType _node = POI, int _degree = 0) : component(_component), id(_id), node(_node), degree(_degree) {}
		PathInfo(PathType _component = EDGE, ID _id = 0, EdgeType _edge = SIDEWALK) : component(_component), id(_id), edge(_edge) {}

		PathType component;
		ID id;
		NodeType node;
		int degree;
		EdgeType edge;

	};


	enum Motion
	{
		GO_FORWARD = 0,		
		STOP,
		TURN
	};

	class Action
	{
	public:
		Action(Motion _do = STOP, int _direction = 0) : move(_do), direction(_direction) {}

		Motion move;
		int direction;		

	};

	/**guide: "until where" "in which direction" "with mode"
	related to "NodeType" "Direction" "EdgeType"
	*/
	class Guide
	{
	public:
		Guide(ID _id, NodeType _type = POI, int _dir = 0, EdgeType _mode = SIDEWALK) : nodeid(_id), type(_type), direction(_dir), mode(_mode) {}

		ID nodeid;
		NodeType type;
		int direction;	//based on robot's past heading
		EdgeType mode;

	};

	class InstantGuide
	{
	public:
		InstantGuide(Guide _guide, Action _action) : guide(_guide), action(_action) {}  

		Guide guide;
		Action action;
	};

	enum Status
	{
		ON_EDGE = 0,
		APPROACHING_NODE,
		ARRIVED_NODE,
		
	};

		
	//For temporary use. To make examples of the path.
	std::list<ID> m_pathnodeids;
	std::list<ID> m_pathedgeids;
	dg::Map m_map;
	bool validatePathNodeIds(); //Validate path ids
	bool savePathNodeEdgeIds(); //Save file of path's node and edge IDs after loading
	bool loadPathFiles(const char* filename, std::list<ID>& destination);	//Load saved path's node and edge IDs file
	std::vector<PathInfo> getPathExample();
	std::vector<dg::TopometricPose> getPositionExample();

	//Guidance member
	std::vector<dg::NodeInfo> m_pathnodes;
	std::vector<dg::EdgeInfo> m_pathedges;
	std::vector<PathInfo> m_path;
	int m_curpathindicator = 0;
	float m_progress = 0;
	Status m_status = ON_EDGE;
	std::vector<Guide> m_guide;
	int m_curguideindicator = 0;;

	Status checkStatus(dg::TopometricPose pose);
	bool generateGuide();
	std::vector<InstantGuide> provideNormalGuide(std::vector<InstantGuide> prevguide, Status status);

};


}

