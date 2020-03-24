#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

bool GuidanceManager::initializeGuides()
{
	if (m_map.nodes.empty() || m_path.pts.size() < 1)
		return false;

	std::vector<PathElement>::iterator iter = m_path.pts.begin();
	std::vector<PathElement>::iterator last = m_path.pts.end();

	m_guides.clear();

	//initial angle
	int angle = 0;
	PathElement tempElement;
	while (iter != last)
	{
		//current node
		tempElement = *iter;
		Node* curNode = tempElement.node;
		Edge* curEdge = tempElement.edge;
		
		//next node
		iter++;
		if (iter == last) goto ADD_LAST_NODE;
		tempElement = *iter;
		Node* nextNode = tempElement.node;

		GuidedPathElement tmppath(curNode, curEdge, nextNode, angle);
		m_guides.push_back(tmppath);

		//calculate degree for next round		
		iter++;	//after next node
		if (iter == last) goto ADD_LAST_NODE;
		tempElement = *iter;
		Node* afterNextNode = tempElement.node;
		angle = getDegree(curNode, nextNode, afterNextNode);
		iter--;

	}//while (curnode != dest_node)
	
ADD_LAST_NODE:

	//last node
	PathElement dest = m_path.pts.back();
	GuidedPathElement tmppath(dest.node, nullptr, nullptr, 0);
	m_guides.push_back(tmppath);

	//set first guide
	Guidance guide = getGuidance(MoveStatus::ON_NODE);
	m_prev_guidance = guide;

	return true;

}

GuidanceManager::MoveStatus GuidanceManager::applyPose(TopometricPose  pose)
{
	//Current robot location
	ID nodeid = pose.node_id;
	int eidx = pose.edge_idx;
	Node* curnode = m_map.findNode(nodeid);
	Edge* curedge = curnode->edge_list[eidx];
	double edgedist = curedge->length;
	double curdist = pose.dist;
	double progress = curdist / edgedist;

	/**Check progress.
	m_edge_progress: 0.0 ~ 1.0
	m_edge_progress indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/

	GuidedPathElement curGP = m_guides[m_guide_idx];

	//Check edge following status
	if ((nodeid != curGP.from_node->id) && (progress < 0.01 ))
	{
		m_mvstatus = MoveStatus::ON_NODE;
	}
	else if (progress >= 0.9)
	{
		m_mvstatus = MoveStatus::APPROACHING_NODE;
	}
	else
	{
		m_mvstatus = MoveStatus::ON_EDGE;
	}

	m_edge_progress = progress;

	return MoveStatus(m_mvstatus);
}


GuidanceManager::Motion GuidanceManager::getMotion(int ntype, int etype, int degree)
{
	std::string rotation;
	if (315 < degree || degree <= 45)
		rotation = "STRAIGHT"; //0 degree
	else if (45 < degree && degree <= 135)
		rotation = "LEFT";
	else if (135 < degree && degree <= 225)
		rotation = "BACK";
	else if (225 < degree && degree <= 315)
		rotation = "RIGHT";
	else
		rotation = "STRAIGHT";

	GuidanceManager::Motion motion;
	if ((ntype == Node::NODE_BASIC) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "STRAIGHT"))
		motion = GuidanceManager::Motion::GO_FORWARD;
	else if (((ntype == Node::NODE_BASIC) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "LEFT")))
		motion = GuidanceManager::Motion::TURN_LEFT;
	else if (((ntype == Node::NODE_BASIC) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "RIGHT")))
		motion = GuidanceManager::Motion::TURN_RIGHT;
	else if (((ntype == Node::NODE_BASIC) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "BACK")))
		motion = GuidanceManager::Motion::TURN_BACK;

	//current heading node is crosswalk
	else if ((ntype == Node::NODE_JUNCTION) && (etype == Edge::EDGE_CROSSWALK)
		&& (rotation == "STRAIGHT"))
		motion = GuidanceManager::Motion::CROSS_FORWARD;
	else if (((ntype == Node::NODE_JUNCTION) && (etype == Edge::EDGE_CROSSWALK)
		&& (rotation == "LEFT")))
		motion = GuidanceManager::Motion::CROSS_LEFT;
	else if (((ntype == Node::NODE_JUNCTION) && (etype == Edge::EDGE_CROSSWALK)
		&& (rotation == "RIGHT")))
		motion = GuidanceManager::Motion::CROSS_RIGHT;

	//current heading node is building
	else if ((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "STRAIGHT"))
		motion = GuidanceManager::Motion::ENTER_FRONT;
	else if (((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "LEFT")))
		motion = GuidanceManager::Motion::ENTER_LEFT;
	else if (((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "RIGHT")))
		motion = GuidanceManager::Motion::ENTER_RIGHT;

	//heading out of building
	else if ((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "STRAIGHT"))
		motion = GuidanceManager::Motion::EXIT_FRONT;
	else if (((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "LEFT")))
		motion = GuidanceManager::Motion::EXIT_LEFT;
	else if (((ntype == Node::NODE_DOOR) && (etype == Edge::EDGE_SIDEWALK)
		&& (rotation == "RIGHT")))
		motion = GuidanceManager::Motion::EXIT_RIGHT;

	else
		motion = GuidanceManager::Motion::UNKNOWN;

	return motion;
	
}


GuidanceManager::Guidance GuidanceManager::getGuidance(MoveStatus status)
{
	Guidance guide;
	GuidedPathElement curGP, nextGP;
	Action curact, nextact;

	//set motion
	switch (status)
	{
	case dg::GuidanceManager::MoveStatus::ON_NODE: 		//add next action
	{
		//update dgstatus
		guide.guide_status = m_dgstatus;
		
		//update moving status
		guide.moving_status = status;
		int gidx = m_guide_idx;
		int gidx_next = gidx+1;

		//update action_current
		curGP = getCurGuidedPath(gidx_next);
		curact.edge_type = curGP.edge->type;
		curact.degree = curGP.degree;
		curact.cmd = getMotion(curGP.to_node->type, curGP.edge->type, curGP.degree);
		curact.mode = getMode(curGP.edge->type);
		guide.action_current = curact;

		//update action_next
		int gidx_thenext = gidx+2;
		if (gidx_thenext == m_guides.size()-1) goto GET_LAST_GUIDE;		//finishing condition
		nextGP = getCurGuidedPath(gidx_thenext);
		nextact.edge_type = nextGP.edge->type;
		nextact.degree = nextGP.degree;
		nextact.cmd = getMotion(nextGP.to_node->type, nextGP.edge->type, nextGP.degree);
		nextact.mode = getMode(nextGP.edge->type);
		guide.action_next = nextact;

		//update heading_node
		guide.heading_node = *curGP.to_node;

		//update distance_to_remain
		guide.distance_to_remain = curGP.edge->length;


		//change member variable
		m_prev_guidance = guide;
		m_guide_idx++;
		
		break;
	}
	case dg::GuidanceManager::MoveStatus::ON_EDGE://maintain current guide
		guide = m_prev_guidance;
		curGP = getCurGuidedPath(m_guide_idx);
		guide.distance_to_remain = curGP.edge->length * (1.0-m_edge_progress);
		break;
	case dg::GuidanceManager::MoveStatus::APPROACHING_NODE://add next action
		guide = m_prev_guidance;
		curGP = getCurGuidedPath(m_guide_idx);
		guide.distance_to_remain = curGP.edge->length * (1.0 - m_edge_progress);
		break;
	default:
		break;
	}

	//make guidance string
	guide.msg = getGuidanceString(guide);
	return guide;

GET_LAST_GUIDE:
	nextact.edge_type = curGP.edge->type;
	nextact.degree = curGP.degree;
	nextact.cmd = Motion::STOP;
	nextact.mode = Mode::MOVE_NORMAL;
	guide.action_next = nextact;
	return guide;

}

//std::vector<GuidanceManager::Guidance> GuidanceManager::getGuidance(MoveStatus status)
//{
//
//	std::vector<Guidance> curGuide;
//	GuidedPathElement initGP = getHeadGuidedPath();
//	Action curAct = setAction(Motion::GO_FORWARD, 0);
//	Action nextAct = setAction(Motion::GO_FORWARD, 0);
//	std::string gstring = getGuidanceString()
//		Guidance initG = setGuidance(GuideStatus::GUIDE_NORMAL,
//			MoveStatus::ON_NODE, curAct, nextAct, Node _nextNode, int _dist,
//			std::string _msg);
//	curGuide.push_back(initG);
//	m_prev_rguide = curGuide;
//	return curGuide;
//
//	std::vector<RobotGuide> result;
//	GuideStruct curG, nextG;
//	ActionStruct act;
//	RobotGuide rg;
//	int gidx = m_guide_idx;
//	GuidedPathElement curGP = m_guides[gidx];
//
//	if (m_guide_idx == m_guides.size() - 2) goto GET_LAST_GUIDE;
//
//	curG = setGuide(curGP.to_node->id, curGP.to_node->type, curGP.edge->type, curGP.degree);
//
//	switch (status)
//	{
//	case MoveStatus::ON_EDGE: //maintain current guide
//		result.push_back(m_prev_rguide.back());
//		break;
//
//	case MoveStatus::APPROACHING_NODE: //add next action
//		result.push_back(m_prev_rguide.back());
//		if (curGP.to_node->type == Node::NODE_JUNCTION)
//		{
//			act = setAction(Motion::STOP, 0);
//			rg = setRobotGuide(curG, act);
//			result.push_back(rg);
//		}		
//		break;
//
//	case MoveStatus::ARRIVED_NODE: //add next action
//	{
//		if (45 < curGP.degree && curGP.degree <= 315)
//		{
//			act = setAction(Motion::TURN, curGP.degree);
//			rg = setRobotGuide(curG, act);
//			result.push_back(rg);
//			
//			act = setAction(Motion::STOP, 0);
//			rg = setRobotGuide(curG, act);
//			result.push_back(rg);
//		}
//
//		gidx++;
//		int idx = gidx;
//		GuidedPathElement nextGP = m_guides[idx];
//		nextG = setGuide(nextGP.to_node->id, nextGP.to_node->type, nextGP.edge->type, nextGP.degree);
//		
//		act = setAction(Motion::GO_FORWARD, 0);
//		rg = setRobotGuide(nextG, act);
//		result.push_back(rg);
//		
//		m_guide_idx++;
//
//		//finishing condition
//		if (m_guide_idx == m_guides.size())
//			return result;
//
//		break;
//	}
//	default:
//		break;
//	}
//
//
//	Guidance guide;
//	guide.guide_status = _gstatus;
//	guide.moving_status = _mstatus;
//	guide.action_current = _curAct;
//	guide.action_next = _nextAct;
//	guide.heading_node = _nextNode;
//	guide.distance_to_remain = _dist;
//	std::string msg = getGuidanceString(_curAct, _action_next, _dist);
//	guide.msg = _msg;
//
//
//	m_prev_rguide = result;
//	
//	return result;
//
//GET_LAST_GUIDE:
//result.push_back(m_prev_rguide.back());
//curG = setGuide(curGP.from_node->id, curGP.from_node->type, 0, 0);
//act = setAction(Motion::STOP, 0);
//rg = setRobotGuide(curG, act);
//result.push_back(rg);
//m_prev_rguide = result;
//return result;
//
//}


std::string GuidanceManager::getGuidanceString(Guidance guidance)
{
	std::string nodetype;
	switch (guidance.heading_node.type)
	{
	case Node::NODE_BASIC:
		nodetype = "POI";
		break;
	case Node::NODE_JUNCTION:
		nodetype = "JUNCTION";
		break;
	case Node::NODE_DOOR:
		nodetype = "DOOR";
		break;
	case Node::NODE_ELEVATOR:
		nodetype = "ELEVATOR";
		break;
	case Node::NODE_ESCALATOR:
		nodetype = "ESCALATOR";
		break;
	default:
		nodetype = "Unknown";
		break;
	}

	std::string motion;
	switch (guidance.action_current.cmd)
	{
	case Motion::GO_FORWARD:
		motion = "GO_FORWARD";
		break;
	case Motion::TURN_LEFT:
		motion = "TURN_LEFT";
		break;
	case Motion::TURN_RIGHT:
		motion = "TURN_RIGHT";
		break;
	case Motion::TURN_BACK:
		motion = "TURN_BACK";
		break;
	case Motion::STOP:
		motion = "STOP";
		break;
	case Motion::CROSS_FORWARD:
		motion = "CROSS_FORWARD";
		break;
	case Motion::CROSS_LEFT:
		motion = "CROSS_LEFT";
		break;
	case Motion::CROSS_RIGHT:
		motion = "CROSS_RIGHT";
		break;
	case Motion::ENTER_FRONT:
		motion = "ENTER_FRONT";
		break;
	case Motion::ENTER_LEFT:
		motion = "ENTER_LEFT";
		break;
	case Motion::ENTER_RIGHT:
		motion = "ENTER_RIGHT";
		break;
	case Motion::EXIT_FRONT:
		motion = "EXIT_FRONT";
		break;
	case Motion::EXIT_LEFT:
		motion = "EXIT_LEFT";
		break;
	case Motion::EXIT_RIGHT:
		motion = "EXIT_RIGHT";
		break;
	case Motion::UNKNOWN:
		motion = "UNKNOWN";
		break;
	default:
		break;
	}


	std::string edge;
	switch (guidance.action_current.edge_type)
	{
	case Edge::EDGE_SIDEWALK:
		edge = "SIDEWALK";
		break;
	case Edge::EDGE_ROAD:
		edge = "ROAD";
		break;
	case Edge::EDGE_CROSSWALK:
		edge = "CROSSWALK";
		break;
	case Edge::EDGE_ELEVATOR:
		edge = "ELEVATOR";
		break;
	case Edge::EDGE_ESCALATOR:
		edge = "ESCALATOR";
		break;
	case Edge::EDGE_STAIR:
		edge = "STAIR";
		break;
	default:
		edge = "Unknown";
		break;
	}

	std::string mode;
	switch (guidance.action_current.mode)
	{
	case Mode::MOVE_NORMAL:
		mode = "MOVE_NORMAL";
		break;
	case Mode::MOVE_CAUTION:
		mode = "MOVE_CAUTION";
		break;
	case Mode::MOVE_CAUTION_CHILDREN:
		mode = "MOVE_CAUTION_CHILDREN";
		break;
	case Mode::MOVE_CAUTION_CAR:
		mode = "MOVE_CAUTION_CAR";
		break;
	default:
		mode = "";
		break;
	}

	std::string nodeid = std::to_string(guidance.heading_node.id);
	std::string degree = std::to_string(guidance.action_current.degree);
	std::string distance = std::to_string(guidance.distance_to_remain);

	std::string result;
	if ((motion == "GO_FORWARD") || (motion == "CROSS_FORWARD")
		|| (motion == "ENTER_FRONT") || (motion == "EXIT_FRONT"))
	{
		result = "[Guide] " + motion + " on " + edge + " about " + distance + "m," + " until next " + nodetype + "(Node ID : " + nodeid + ")";
	}
	else if (motion == "STOP")
	{
		result = "[Guide] STOP at " + nodetype + "(Node ID : " + nodeid + ")";
	}
	else if ((motion == "TURN_LEFT") || (motion == "TURN_RIGHT")
		|| (motion == "TURN_BACK") || (motion == "CROSS_LEFT")
		|| (motion == "CROSS_RIGHT") || (motion == "ENTER_LEFT")
		|| (motion == "ENTER_RIGHT") || (motion == "EXIT_LEFT")
		|| (motion == "EXIT_RIGHT"))
	{
		result = "[Guide] " + motion + " on " + degree + " on " + nodetype + " (Node ID : " + nodeid + ")";
	}
	else
	{		
		result = "[Guide] " + motion + " on " + edge + " until next " + nodetype + "(Node ID : " + nodeid + ")";
	}

	return result;
}

/** test code
	int t1 = m_guider.getDegree(1.0, 1.0, 1.0, 0.0, 2.0, 1.0);
	printf("t1: %d\n", t1);
	int t2 = m_guider.getDegree(1.0, 1.0, 1.0, 0.0, 1.0, 1.0);
	printf("t2: %d\n", t2);
	int t3 = m_guider.getDegree(1.0, 1.0, -1.0, 0.0, -1.0, -1.0);
	printf("t3: %d\n", t3);
	int t4 = m_guider.getDegree(1.0, 1.0, -1.0, 0.0, -1.0, 1.0);
	printf("t4: %d\n", t4);
	int Guidance::getDegree(double x1, double y1, double x2, double y2, double x3, double y3)

	Map::Node* foundNode1 = map.findNode(559542564800587);
	Map::Node* foundNode2 = map.findNode(559542564800586);
	Map::Node* foundNode3 = map.findNode(559542564800055);
	Map::Node* foundNode4 = map.findNode(559542564800056);
	Map::Node* foundNode5 = map.findNode(559552564800620);

	int angle = m_guider.getDegree(&foundNode1->data, &foundNode2->data, &foundNode3->data);
	printf("Node: %llu --> Angle is: %d\n", foundNode1->data.id, angle);
*/
int GuidanceManager::getDegree(Node* node1, Node* node2, Node* node3)
{
	double x1 = node1->lon;
	double y1 = node1->lat;
	double x2 = node2->lon;
	double y2 = node2->lat;
	double x3 = node3->lon;
	double y3 = node3->lat;

	double v1x = x2 - x1;
	double v1y = y2 - y1;
	double v2x = x3 - x2;
	double v2y = y3 - y2;

	if (!(v1x * v1x + v1y * v1y) || !(v2x * v2x + v2y * v2y))
	{
		int result = 0;
		return result;
	}

	double sign = asin((v1x * v2y - v1y * v2x) / (sqrt(v1x * v1x + v1y * v1y) * sqrt(v2x * v2x + v2y * v2y)));
	double rad = acos((v1x * v2x + v1y * v2y) / (sqrt(v1x * v1x + v1y * v1y) * sqrt(v2x * v2x + v2y * v2y)));

	double rad2deg = rad / 3.14 * 180.0;
	if (sign < 0.f) rad2deg = 360.f - rad2deg;

	int result = (int) rad2deg;

	return result;

}


