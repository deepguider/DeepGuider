#pragma once
#include "dg_core.hpp"

namespace dg
{

class Guidance
{
public:

	Guidance()	{ }
	
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
		ID node_id;

		/**The type of heading node*/
		int node_type;

		/**The robot moving mode depends on street(edge) type*/
		int edge_type;

		/**Turning direction of robot on the node  based on robot's heading*/
		int degree;

	};


	/**A realtime action of robot including motion and direction */
	struct ActionStruct
	{
		Motion move;
		double distance;
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

	/**@brief A segment of guided path
	*/
	class GuidedPathElement
	{
	public:
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

	std::list<ID> m_path_nodes;
	std::list<ID> m_path_edges;
	//int m_curPathNodeIdx = 0;
	int m_guide_idx = 0;
	double m_edge_progress = 0;
	std::vector<RobotGuide> m_prev_rguide;
	MoveStatus  m_mvstatus = MoveStatus::ON_EDGE;
	DGStatus  m_dgstatus = DGStatus::OK;

	int getDegree(dg::Node* node1, dg::Node* node2, dg::Node* node3);

	void printSingleRGuide(RobotGuide rGuide);
	
public:
	bool setPathNMap(dg::Path path, dg::Map map)
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

	GuideStruct setGuide(ID _nodeid, int _nodetype, int _edgetype, int _degree)
	{
		GuideStruct guide;
		guide.node_id = _nodeid;
		guide.node_type = _nodetype;
		guide.edge_type = _edgetype;
		guide.degree = _degree;
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

	GuidedPathElement getHeadGuidedPath() { return m_guides[0]; }

	std::vector<RobotGuide> getInitGuide()
	{
		std::vector<RobotGuide> curGuide;
		GuidedPathElement initGP = getHeadGuidedPath();
		GuideStruct initG = setGuide(initGP.to_node->id, initGP.to_node->type, initGP.edge->type, initGP.degree );
		ActionStruct initA = setAction(Motion::GO_FORWARD, 0);
		RobotGuide initRG = setRobotGuide(initG, initA);
		curGuide.push_back(initRG);
		m_prev_rguide = curGuide;
		return curGuide;
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

};


}

