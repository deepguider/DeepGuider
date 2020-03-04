#pragma once
#include "dg_core.hpp"

namespace dg
{

class Guidance
{
public:

	Guidance()	{ }

	/** The type of the node*/
	//Map::Node::NodeType Node;

	enum class NodeEnum
	{
		NT_BS = 0,	//POI on continuous sidewalk, streets are connected with 180 degree
		NT_JT,		//Junction, Island, Road end, etc. streets are connected with 90 DEGREE
		NT_DR,	//Entrance of a building
		NT_EV,	//In front of an elevator
		NT_ES	//In fron of an escalator
	};

	/** The type of the edge on which the robot moves*/
	enum class EdgeEnum
	{
		ET_SD = 0,	//sidewalk, Street for pedestrian
		ET_MD,	//road. no sidewalk
		ET_CR, //Pedestrian Crosswalk with traffic light
		ET_DR, //connecting inside and outside
		ET_EV,	//connecting floors with an elevator
		ET_ES		//connecting floors with an escalator
	};
	
	/** Robot's moving status*/
	enum class MoveStatus
	{
		ON_EDGE = 0,	//The robot is 0~90% of the edge distance
		APPROACHING_NODE,	//The robot is 90~99% of the edge distance
		ARRIVED_NODE		//The robot has arrived at the node
	};

	/** Robot status*/
	enum class RobotStatus
	{
		OK = 0,
		NOK_RB,	//No way. Road blocked
		NOK_UK	//Unknown failure
	};

	/** DG status*/
	enum class DGStatus
	{
		OK = 0,	//normal condition
		LOST,	//The localizer normal condition
		NOK	//failure for any reason
	};

	/**A motion of robot for guidance*/
	enum class Motion
	{
		GO_FORWARD = 0,
		STOP,
		TURN
	};
	   	  
	/**@brief Initial guide derived from path
	*
	* Guide component: "until where" "in which direction" "with mode"
	* Each of guide component is related to "NodeType" "Direction" "EdgeType"
	*/
	struct GuideStruct
	{
		/** ID of NODE which the robot is heading ahead*/
		ID nodeid;

		/**The type of heading node*/
		NodeEnum nodetype;

		/**The robot moving mode depends on street(edge) type*/
		EdgeEnum edgetype;

		/**Turning direction of robot on the node  based on robot's heading*/
		int degree;

	};


	/**An action of robot including motion and direction */
	struct ActionStruct
	{
		Motion move;
		int degree;
	};


	/**@brief Detailed guide for each status while robot is moving
	*/
	struct RobotGuide
	{
		/** A segment of generated guide sequence	*/
		GuideStruct guide;

		/** Detail action of each guided node*/
		ActionStruct action;
	};


private:
		
	/**Type of path, whether the path is node or edge.*/
	enum class PathEnum
	{
		NODE = 0,
		EDGE
	};

	/**@brief A segment of guided path
	*/
	class GuidedPathType
	{
	public:
		/** A constructor with path's segment
		* @param _component An option whether the path segment is NODE or EDGE
		* @param _id The given ID
		* @param _node An element from NodeType
		* @param _degree An relative orientation from past node to current node
		* @param _edge And element from EdgeType
		*/
		GuidedPathType(ID _curnid = 0, NodeEnum _curntype = NodeEnum::NT_BS,
			ID _eid = 0, EdgeEnum _etype = EdgeEnum::ET_SD,
			ID _nextnid = 0, NodeEnum _nextntype = NodeEnum::NT_BS,
			int _degree = 0)
			: curnid(_curnid), curntype(_curntype), eid(_eid), etype(_etype), 
			nextnid(_nextnid), nextntype(_nextntype), degree(_degree) {}

		//GuidedPathType(PathEnum _component = PathEnum::NODE, ID _id = 0, NodeEnum _node = NodeEnum::NT_BS, int _degree = 0) : component(_component), id(_id), nodetype(_node), degree(_degree) {}
		//GuidedPathType(PathEnum _component = PathEnum::EDGE, ID _id = 0, EdgeEnum _edge = EdgeEnum::ET_SD) : component(_component), id(_id), edgetype(_edge) {}


		/** Either Node or EDGE */
		//PathEnum component;
		
		/** Current ID of NODE*/
		ID curnid;

		/** Current Type of Node*/
		NodeEnum curntype;
		
		/** ID of EDGE*/
		ID eid;

		/** Type of Edge*/
		EdgeEnum etype;

		/** Next ID of NODE*/
		ID nextnid;

		/** Next Type of Node*/
		NodeEnum nextntype;

		/** Rotation angle at next node. [deg] 0~359*/
		int degree;

	};

	//Guidance member
	dg::Path m_path;
	dg::Map m_map;
	std::vector <GuidedPathType> m_initGuides;

	std::list<ID> m_pathNodeIds;
	std::list<ID> m_pathEdgeIds;
	//int m_curPathNodeIdx = 0;
	int m_curGuideIdx = 0;
	double m_edgeElapse = 0;
	std::vector<RobotGuide> m_prevguide;
	MoveStatus  m_mvstatus = MoveStatus::ON_EDGE;
	DGStatus  m_dgstatus = DGStatus::OK;

	int getDegree(dg::NodeInfo* node1, dg::NodeInfo* node2, dg::NodeInfo* node3);
	//int getDegree(double x1, double y1, double x2, double y2, double x3, double y3);

	void printSingleRGuide(RobotGuide rGuide);
	
public:
	//for Guidance use
	bool setPathNMap(dg::Path path, dg::Map map)
	{
		if (path.countPoints() < 1)
		{
			fprintf(stdout, "No path input!\n");
			return false;
		}
		m_path = path;
		m_map = map;
		return true;
	}

	GuideStruct setGuide(ID nodeid, NodeEnum nodetype, EdgeEnum edgetype, int degree)
	{
		GuideStruct guide;
		guide.nodeid = nodeid;
		guide.nodetype = nodetype;
		guide.edgetype = edgetype;
		guide.degree = degree;
		return guide;
	}

	ActionStruct setAction(Motion _do, int _degree)
	{
		ActionStruct action;
		action.move = _do;
		action.degree = _degree;
		return action;
	}

	RobotGuide setRobotGuide(GuideStruct _guide, ActionStruct _action)
	{
		RobotGuide guide;
		guide.guide = _guide;
		guide.action = _action;
		return guide;
	}

	GuidedPathType getHeadGuidedPath() { return m_initGuides[0]; }

	bool setInitRobotGuide()
	{
		bool result = false;
		std::vector<RobotGuide> curGuide;
		GuidedPathType initGP = getHeadGuidedPath();
		GuideStruct initG = setGuide(initGP.nextnid, initGP.nextntype, initGP.etype, initGP.degree );
		ActionStruct initA = setAction(Motion::GO_FORWARD, 0);
		RobotGuide initRG = setRobotGuide(initG, initA);
		curGuide.push_back(initRG);
		m_prevguide = curGuide;
		result = true;
		return result;
	}

	bool initializeGuides();
	MoveStatus applyPose(dg::TopometricPose pose);
	DGStatus getRobotStatus(RobotStatus rs, MoveStatus ms);
	std::vector<RobotGuide> getNormalGuide(MoveStatus status);
	std::vector<RobotGuide> getActiveExplorGuide();
	std::vector<RobotGuide> getOptimalExplorGuide();
	void printRobotGuide(std::vector<RobotGuide> rGuides);

	//for temporary use
	bool loadLocFiles(const char* filename, std::vector<dg::TopometricPose>& destination);

private:

	//For temporary use. To make examples of the path.
	//std::list<ID> m_pathedgeids;
	//std::vector<dg::NodeInfo> m_pathnodes;
	//std::vector<dg::EdgeInfo> m_pathedges;
	//std::vector<GuideStruct> m_guides;

	//bool generateGuide();
	//bool validatePathNodeIds(); //Validate path ids
	//bool savePathNodeEdgeIds(); //Save file of path's node and edge IDs after loading
	//bool loadPathFiles(const char* filename, std::list<ID>& destination);	//Load saved path's node and edge IDs file
	//bool loadPathFiles(const char* filename, std::vector<GuidedPathType>& destination);
	
};


}

