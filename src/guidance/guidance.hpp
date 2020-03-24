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
		ENTER_FRONT,
		ENTER_LEFT,
		ENTER_RIGHT,
		EXIT_FRONT,
		EXIT_LEFT,
		EXIT_RIGHT,

		UNKNOWN
	};

	// motion mode	
	enum class Mode
	{
		MOVE_NORMAL,
		MOVE_CAUTION,
		MOVE_CAUTION_CHILDREN,
		MOVE_CAUTION_CAR,
	};

	/**A realtime action of robot including motion and direction */
	struct Action
	{
		Motion cmd;	// action command
		int edge_type;	//
		int degree = 0;	// additional action direction of current cmd
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
		Action action_current;	// action command to next branch node
		Action action_next;		// action command after passing branch node
		Node heading_node;	 // heading node
		double distance_to_remain; // distance to heading node (unit: meter)
		std::string msg;		// string guidance message
	};
	
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
		GuidedPathElement(Node* _fromnode, Edge* _edge,
			Node* _tonode, int _degree = 0)
			: from_node (_fromnode), edge(_edge), to_node(_tonode), degree(_degree) {}

		/** Current NODE*/
		Node* from_node;

		/** EDGE*/
		Edge* edge;

		/** Next NODE*/
		Node* to_node;

		/** Rotation angle at next node. [deg] 0~359*/
		int degree;

	};

	//Guidance member
	dg::Path m_path;
	dg::Map m_map;
	std::vector <GuidedPathElement> m_guides;
	int m_guide_idx = -1;
	double m_edge_progress = 0;

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

	MoveStatus  m_mvstatus = MoveStatus::ON_EDGE;
	GuideStatus  m_dgstatus = GuideStatus::GUIDE_NORMAL;
	Guidance m_prev_guidance;

	Motion getMotion(int ntype, int etype, int degree);
	int getDegree(dg::Node* node1, dg::Node* node2, dg::Node* node3);
	GuidedPathElement getCurGuidedPath(int idx) { return m_guides[idx]; }

	
public:
	bool setPathNMap(dg::Path path, dg::Map& map)
	{
		if (path.pts.size() < 1)
		{
			fprintf(stdout, "No path input!\n");
			return false;
		}
		m_path = path;
		m_map = map;
		return true;
	}

	bool initializeGuides();
	MoveStatus applyPose(dg::TopometricPose pose);
	GuideStatus getGuidanceStatus(GuideStatus rs, MoveStatus ms);
	//GuideStatus getStatus(GuideStatus rs, MoveStatus ms);
	Guidance getGuidance(MoveStatus status);

	std::string getGuidanceString(Guidance guidance);

};


}

