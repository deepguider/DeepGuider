#pragma once
#include "dg_core.hpp"

namespace dg
{

class Guidance
{
public:
	/** A nodetype where the node is appointed*/
	enum NodeType
	{
		POI = 0,	//Continuous sidewalk, streets are connected with 180 degree
		CORNER,		//Junction, streets are connected with 90 DEGREE
		ISLAND,		//Surrounded by road. streets are not connected
		ROAD_END	//End of the sidewalk
	};

	/** An edgetype on which robot moves*/
	enum EdgeType
	{
		SIDEWALK = 0,	//Street for pedestrian
		CROSSWALKWOLIGHT,	//Pedestrian crosswalk without traffic light
		CROSSWALKWLIGHT,	//Pedestrian Crosswalk with traffic light
		CROSSROAD,	//Crossing roadway without traffic light
		ROADWAY		//Street for car
	};

	/** Type of path, whether the path is node or edge. */
	enum PathType
	{
		NODE = 0,
		EDGE
	};

	/**@brief Path's segment
	*
	* The path is consisted with node and edge information.	
	*/
	class PathInfo
	{
	public:
		/** A constructor with path's segment
		* @param _component An option whether the path segment is NODE or EDGE
		* @param _id The given ID
		* @param _node An element from NodeType
		* @param _degree An relative orientation from past node to current node
		* @param _edge And element from EdgeType
		*/
		PathInfo(PathType _component = NODE, ID _id = 0, NodeType _node = POI, int _degree = 0) : component(_component), id(_id), node(_node), degree(_degree) {}
		PathInfo(PathType _component = EDGE, ID _id = 0, EdgeType _edge = SIDEWALK) : component(_component), id(_id), edge(_edge) {}

		/** Either Node or EDGE */
		PathType component;

		/** ID of NODE or EDGE*/
		ID id;

		/** Type of Node*/
		NodeType node;

		/** Degree between past node to current node. Unit: [deg]*/
		int degree;

		/** Type of Edge*/
		EdgeType edge;

	};

	/**A motion of robot for guidance*/
	enum Motion
	{
		GO_FORWARD = 0,		
		STOP,
		TURN
	};

	/**An action of robot including motion and direction */
	class Action
	{
	public:
		Action(Motion _do = STOP, int _direction = 0) : move(_do), direction(_direction) {}

		Motion move;
		int direction;		

	};

	/**@brief General guide for robot
	* 
	* General guide derived from path 
	* Guide component: "until where" "in which direction" "with mode"
	* Each of guide component is related to "NodeType" "Direction" "EdgeType"
	*/
	class Guide
	{
	public:
		/** A constructor of guide
		*
		* @param _id Node ID
		* @param _type Nodetype
		* @param _dir Turn in direction
		* @param _mode In which street type the robot moves
		*/
		Guide(ID _id, NodeType _type = POI, int _dir = 0, EdgeType _mode = SIDEWALK) : nodeid(_id), type(_type), direction(_dir), mode(_mode) {}

		/** ID of NODE which the robot is heading ahead*/
		ID nodeid;

		/**The type of heading node*/
		NodeType type;

		/**Turning direction of robot on the node*/
		int direction;	//based on robot's past heading

		/**The robot moving mode depends on street(edge) type*/
		EdgeType mode;

	};

	/**@brief Detailed guide for each step 
	* 
	* An instant action is added to guide
	*/
	class InstantGuide
	{
	public:
		/** A constructor for instant guide
		* @param guide A part from guide sequence
		* @param action A instant action for each guide step
		*/
		InstantGuide(Guide _guide, Action _action) : guide(_guide), action(_action) {}  

		/** A segment of generated guide sequence	*/
		Guide guide;

		/** Detail action of each guided node*/
		Action action;
	};

	/** Robot's moving status*/
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
	bool loadPathFiles(const char* filename, std::vector<PathInfo>& destination);
	bool loadLocFiles(const char* filename, std::vector<dg::TopometricPose>& destination);

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

