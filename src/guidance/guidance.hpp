#pragma once
#include <time.h>
#include <chrono>
#include "dg_core.hpp"
#include "dg_map_manager.hpp"

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
		GUIDE_OOP_DETECT,		//may be out of path. let's see
		GUIDE_OOP,		//out of path, localization confidence is high
		GUIDE_LOST,	//localization confidence is low

		//from exploration control module
		GUIDE_RECOVERY,	//return to previous known POI
		GUIDE_EXPLORATION, 	//find near POI
		GUIDE_OPTIMAL_VIEW,	//find optimal viewpoint to look POI

		//from robot
		GUIDE_LOW_BATTERY,	//Low battery
		GUIDE_PATH_BLOCKED, 	//Path exist, but temporary blocked
		GUIDE_NO_PATH,		//Path not exist as map
		GUIDE_BAD_GROUND,	//Uneven ground
		GUIDE_ROBOT_FAIL, 	//Robot failure
		GUIDE_FAIL_UNKNOWN,	//Unknown failure

		GUIDE_INITIAL,
		GUIDE_UNKNOWN
	};

	/** Moving status based on localization info*/
	enum class MoveStatus
	{
		ON_NODE = 0,		//The robot has arrived at the node
		ON_EDGE,	//The robot is 0~90% of the edge distance
		APPROACHING_NODE,	//The robot is 90~99% of the edge distance
		STOP_WAIT	//not moving
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
		double distance; //distance for active exploration
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
		bool announce = 1;
	};
	
//private:		

	/**@brief A segment of guided path
	*/
	struct GuidedPathElement
	{
		GuidedPathElement() {};
		/** A constructor with path's segment
		* @param _Current NODE
		* @param _Current EDGE
		* @param _Next NODE
		* @param _Next EDGE
		* @param _Degree of rotation angle at next node. [deg] -180~180
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

	//Guidance member
	dg::Path m_path;
	dg::Map m_map;
	dg::LatLon m_latlon;
	std::vector <GuidedPathElement> m_guides;
	int m_guide_idx = -1;	//starts with -1 because its pointing current guide.
	double m_edge_progress = 0.0;
	double m_rmdistance = 0.0;
	MoveStatus  m_mvstatus = MoveStatus::ON_EDGE;
	GuideStatus  m_gstatus = GuideStatus::GUIDE_NORMAL;
	Guidance m_curguidance;
	std::vector<Guidance> m_past_guides;
	time_t oop_start = 0 , oop_end = 0;

	std::string m_mvstring[4] = {"ON_NODE","ON_EDGE", "APPROACHING_NODE", "ARRIVED"};
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

	bool isNodeInPath(ID nodeid);
	Motion getMotion(int ntype, int etype, int degree);
	Guidance getLastGuidance() { return m_past_guides.back(); };
	std::string getStringAction(Action action);
	std::string getStringForward(Action act, int ntype, ID nid, double d);
	std::string getStringTurn(Action act, int ntype);
	std::string getStringTurnDist(Action act, int ntype, double dist);
	std::string getStringGuidance(Guidance guidance, MoveStatus status);
	GuidedPathElement getCurGuidedPath(int idx) { return m_guides[idx]; };
	int getGuideIdxFromPose(TopometricPose pose);
	int getDegree(dg::Node* node1, dg::Node* node2, dg::Node* node3);
	Map getMap() { return m_map; };
	MoveStatus getMoveStatus() { return m_mvstatus; };
	LatLon getPoseGPS() { return m_latlon; };
	bool regeneratePath(double start_lat, double start_lon, double dest_lat, double dest_lon);

	//bool setGuidance(TopometricPose pose);
	//bool setGuideStatus(GuideStatus gs) { m_gstatus = gs; };
	bool setGuidanceStatus(TopometricPose pose, double conf);
	bool setGuidanceWithGStatus();
	bool setInitialGuide();
	bool setNormalGuide();
	bool setOOPGuide();
	bool setArrivalGuide();
	bool setEmptyGuide();
	bool applyPose(TopometricPose pose);
	Action setActionCmd(Motion ncmd, int etype, int degree, Mode mode);
	Action setAction(ID nid, ID eid, int degree);
	Action addTurnAction(GuidedPathElement gp);
		
public:

	bool setPathNMap(dg::Path path, dg::Map map);
	bool initializeGuides();
	bool validatePath(dg::Path path, dg::Map map);
	bool update(TopometricPose pose, double confidence);
	GuideStatus getGuidanceStatus() const { return m_gstatus; }; 
	Guidance getGuidance() const { return m_curguidance; };
	bool applyPoseGPS(LatLon gps);

	//makeLostValue related variable and method
	double m_prevconf = 1.0;
	double m_lostvalue = 0.0;
	void makeLostValue(double prevconf, double curconf);
};


}
