#pragma once
#include "dg_core.hpp"

namespace dg
{

class GuidanceManager
{
public:

	GuidanceManager()	{ }

	/** DeepGuider status*/
	enum class GuideStatus
	{
		//from localizer info
		GUIDE_NORMAL,
		GUIDE_ARRIVED,		//finally arrived 
		GUIDE_LOST,		//out of path

		//from lost control module
		GUIDE_EXPLORATION, 	//from exploration module
		GUIDE_RECOVERY_MODE,	//from recovery module
		GUIDE_OPTIMAL_VIEW,	//from optimal view

		//from robot
		GUIDE_LOW_BATTERY,	//Low battery
		GUIDE_PATH_BLOCKED, 	//Path exist, but temporary blocked
		GUIDE_NO_PATH,		//Path not exist as map
		GUIDE_BAD_GROUND,	//Uneven ground
		GUIDE_ROBOT_FAIL, 	//Robot failure
		GUIDE_FAIL_UNKNOWN,	//Unknown failure
	};

	/** Moving status based on localization info*/
	enum class MoveStatus
	{
		ON_NODE = 0,		//The robot has arrived at the node
		ON_EDGE,	//The robot is 0~90% of the edge distance
		APPROACHING_NODE,	//The robot is 90~99% of the edge distance
		ARRIVED
		//STOP_WAIT	//not moving
	};

	/**A motion of robot for guidance*/
	enum class Motion
	{
		//on sidewalk
		GO_FORWARD = 0,
		TURN_LEFT,
		TURN_RIGHT,
		TURN_BACK,	//in case of lost
		STOP,	//in front of crosswalk

		//on crosswalk
		CROSS_FORWARD,
		CROSS_LEFT,
		CROSS_RIGHT,

		//building
		ENTER_FORWARD,
		ENTER_LEFT,
		ENTER_RIGHT,
		EXIT_FORWARD,
		EXIT_LEFT,
		EXIT_RIGHT
	};

	// motion mode	
	enum class Mode
	{
		MOVE_NORMAL,
		MOVE_CAUTION,
		MOVE_CAUTION_CHILDREN,
		MOVE_CAUTION_CAR,
	};

	/**A realtime action of robot including motion and direction
	* Example:
	* - Go: [GO_**] on [edge_type] with [Mode]
	* - TURN: [TURN_**] for [degree] degree on [node_type]*/
	struct Action
	{
		Motion cmd;	// action command
		int node_type;
		int edge_type;	//
		int degree = 0;	// additional action direction of turn cmd -180~180
		Mode mode = Mode::MOVE_NORMAL;	// addition motion info
	};

	/**@brief Detailed guide for each status while robot is moving
	* Guide component: "until where" "in which direction" "with mode"
	* Each of guide component is related to "NodeType" "Direction" "EdgeType"
	*/
	struct Guidance
	{
		GuideStatus guide_status;	// current guidance status
		MoveStatus moving_status;	// current moving status
		std::vector<Action> actions;	// action command
		ID heading_node_id;	 // heading node
		double distance_to_remain; // distance to heading node (unit: meter)
		std::string msg;		// string guidance message
		bool announce = 0;
	};
	//struct Guidance
	//{
	//	GuideStatus guide_status;	// current guidance status
	//	MoveStatus moving_status;	// current moving status
	//	std::vector<Action> actions;	// action command
	//	Node heading_node;	 // heading node
	//	double distance_to_remain; // distance to heading node (unit: meter)
	//	std::string msg;		// string guidance message
	//	bool announce = 0;
	//};
	
private:		

	/**@brief A segment of guided path
	*/
	struct GuidedPathElement
	{
		GuidedPathElement() {};
		/** A constructor with path's segment
		* @param _component An option whether the path segment is NODE or EDGE
		* @param _id The given ID
		* @param _node An element from NodeType
		* @param _degree An relative orientation from past node to current node
		* @param _edge And element from EdgeType
		*/
		GuidedPathElement(ID _fromnode, ID _curedge, ID _tonode,
			ID _nextedge, int _degree = 0)
			: from_node_id(_fromnode), cur_edge_id(_curedge), to_node_id(_tonode),
			next_edge_id(_nextedge), degree(_degree) {}

		/** Current NODE*/
		ID from_node_id;

		/** Current EDGE*/
		ID cur_edge_id;

		/** Next NODE*/
		ID to_node_id;

		/** Next EDGE*/
		ID next_edge_id;

		/** Rotation angle at next node. [deg] -180~180*/
		int degree;

	};
	//struct GuidedPathElement
	//{
	//	GuidedPathElement() {};
	//	/** A constructor with path's segment
	//	* @param _component An option whether the path segment is NODE or EDGE
	//	* @param _id The given ID
	//	* @param _node An element from NodeType
	//	* @param _degree An relative orientation from past node to current node
	//	* @param _edge And element from EdgeType
	//	*/
	//	GuidedPathElement(Node _fromnode, Edge _curedge, Node _tonode, 
	//		Edge _nextedge, int _degree = 0)
	//		: from_node (_fromnode), cur_edge(_curedge), to_node(_tonode), 
	//		next_edge(_nextedge), degree(_degree) {}

	//	/** Current NODE*/
	//	ID from_node;

	//	/** Current EDGE*/
	//	Edge cur_edge;

	//	/** Next NODE*/
	//	Node to_node;

	//	/** Next EDGE*/
	//	Edge next_edge;

	//	/** Rotation angle at next node. [deg] -180~180*/
	//	int degree;

	//};

	//Guidance member
	dg::Path m_path;
	dg::Map m_map;
	std::vector <GuidedPathElement> m_guides;
	int m_guide_idx = -1;	//starts with -1 because its pointing current guide.
	double m_edge_progress = 0;
	MoveStatus  m_mvstatus = MoveStatus::ON_EDGE;
	GuideStatus  m_dgstatus = GuideStatus::GUIDE_NORMAL;
	std::vector<Guidance> m_past_guides;
	std::string m_nodes[6] = { "POI", "JUNCTION", "DOOR", "ELEVATOR"
		"ESCALATOR", "UNKNOWN" };
	std::string m_edges[7] = { "SIDEWALK", "ROAD", "CROSSWALK", "ELEVATOR",
		"ESCALATOR", "STAIR", "UNKNOWN" };
	std::string m_motions[15] = { "GO_FORWARD", "TURN_LEFT", "TURN_RIGHT",
		"TURN_BACK", "STOP", "CROSS_FORWARD", "CROSS_LEFT",
		"CROSS_RIGHT", "ENTER_FORWARD", "ENTER_LEFT", "ENTER_RIGHT",
		"EXIT_FORWARD", "EXIT_LEFT", "EXIT_RIGHT", "UNKNOWN" };
	std::string m_modes[4] = { "MOVE_NORMAL", "MOVE_CAUTION", "MOVE_CAUTION_CHILDREN",
		"MOVE_CAUTION_CAR" };

	Mode getMode(int etype)
	{
		Mode mode;
		switch (etype)
		{
		case dg::Edge::EDGE_ROAD:
			mode = Mode::MOVE_CAUTION_CAR;
			break;
		case dg::Edge::EDGE_CROSSWALK:
			mode = Mode::MOVE_CAUTION;
			break;
		default:
			mode = Mode::MOVE_NORMAL;
			break;
		}
		return mode;
	}
	
	bool isNodeInPath(ID nodeid);
	Action setAction(Motion ncmd, int etype, int degree, Mode mode);
	Action setAction(ID nid, ID eid, int degree);
	bool isForward(int degree)
	{
		return (degree >= -45 && degree <= 45) ? true : false;
	};

	bool isForward(Motion motion)
	{
		return ((motion == Motion::GO_FORWARD) ||
			(motion == Motion::CROSS_FORWARD) ||
			(motion == Motion::ENTER_FORWARD) ||
			(motion == Motion::EXIT_FORWARD))
			? true : false;
	};

	bool setGuidance();
	Motion getMotion(int ntype, int etype, int degree);
	int getDegree(dg::Node* node1, dg::Node* node2, dg::Node* node3);
	GuidedPathElement getCurGuidedPath(int idx) { return m_guides[idx]; };
	Guidance getLastGuidance() { return m_past_guides.back(); };
	std::string getActionString(Action action);
	std::string getForwardString(Action act, int ntype, ID nid, double d);
	std::string getTurnString(Action act, int ntype);
	int getGuideIdxFromPose(TopometricPose pose);
	
	//Node* findNodeFromPath(ID nodeid);
	//Edge* findEdgeFromPath(ID edgeid);
	
	
public:

	bool setPathNMap(dg::Path path, dg::Map map)
	{
		if (path.pts.size() < 1)
		{
			fprintf(stdout, "No path input!\n");
			return false;
		}
		if (!validatePath(path, map))
		{
			fprintf(stdout, "Path id is not in map!\n");
			return false;
		}
		m_path = path;
		m_map = map;
		return true;
	}

	bool validatePath(dg::Path path, dg::Map map);
	bool initializeGuides();
	bool regeneratePath(TopometricPose pose);
	MoveStatus applyPose(TopometricPose pose);
	MoveStatus applyPose(TopometricPose pose, double confidence);
	GuideStatus getGuidanceStatus(GuideStatus rs, MoveStatus ms);
	//GuideStatus getStatus(GuideStatus rs, MoveStatus ms);
	void setGuideStatus(GuideStatus gs) { m_dgstatus = gs; };
	Guidance getGuidance(MoveStatus status);
	Guidance getGuidance(TopometricPose pose);

	std::string getGuidanceString(Guidance guidance, MoveStatus status);

};


}

