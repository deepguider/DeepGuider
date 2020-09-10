#include "guidance.hpp"
#include "dg_map_manager.hpp"

#define PI 3.141592

using namespace dg;

bool GuidanceManager::initiateNewGuidance(Path& path, Map& map)
{
	if (path.pts.size() < 1)
	{
		printf("[Error] GuidanceManager::initiateNewGuidance - No path input!\n");
		return false;
	}
	if (!validatePath(path, map))
	{
		printf("[Error] GuidanceManager::initiateNewGuidance - Path id is not in map!\n");
		return false;
	}
	m_path = path;
	m_map = map;

	return buildGuides();
}


bool GuidanceManager::buildGuides()
{
	if (m_map.nodes.empty())
	{
		printf("[Error] GuidanceManager::buildGuides - Empty Map\n");
		return false;
	}
	if (m_path.pts.size() < 1)
	{
		printf("[Error] GuidanceManager::buildGuides - Empty path\n");
		return false;
	}

	m_extendedPath.clear();

	for (int i = 0; i < (int)m_path.pts.size() - 1; i++)
	{
		ID curnid = m_path.pts[i].node_id;
		ID cureid = m_path.pts[i].edge_id;
		ID nextnid = m_path.pts[i + 1].node_id;
		ID nexteid = m_path.pts[i + 1].edge_id;

		Node* curNode = m_map.findNode(curnid);
		if (curNode == nullptr)
		{
			printf("[Error] GuidanceManager::buildGuides()\n");
			return false;
		}

		int angle = 0;
		if (i > 0)
		{
			ID prevnid = m_path.pts[i - 1].node_id;
			Node* prevNode = m_map.findNode(prevnid);
			Node* nextNode = m_map.findNode(nextnid);

			angle = getDegree(prevNode, curNode, nextNode);
		}
		ExtendedPathElement tmppath(curnid, cureid, nextnid, nexteid, angle);
		if (curNode->type == Node::NODE_JUNCTION || curNode->type == Node::NODE_DOOR)
		{
			tmppath.is_junction = true;
		}
		m_extendedPath.push_back(tmppath);
	}

	//add last node
	Node* lastNode = m_map.findNode(m_path.pts.back().node_id);
	ID lastEdge = lastNode->edge_ids[0];
	m_finalTurn = 0;
	m_extendedPath.push_back(ExtendedPathElement(m_path.pts.back().node_id, lastEdge, 0, 0, m_finalTurn));

	// update remain distance to next junction
	double d_accumulated = 0;
	for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
	{
		ID eid = m_extendedPath[i].cur_edge_id;
		Edge* edge = m_map.findEdge(eid);
		d_accumulated += edge->length;

		m_extendedPath[i].remain_distance_to_next_junction = d_accumulated;

		if (m_extendedPath[i].is_junction)
		{
			d_accumulated = 0;
		}
	}

	// update past distance from prev junction
	d_accumulated = 0;
	for (int i = 1; i < (int)m_extendedPath.size(); i++)
	{
		ID eid = m_extendedPath[i - 1].cur_edge_id;
		Edge* edge = m_map.findEdge(eid);
		d_accumulated += edge->length;

		m_extendedPath[i].past_distance_from_prev_junction = d_accumulated;

		if (m_extendedPath[i].is_junction)
		{
			d_accumulated = 0;
		}
	}

	// update remain distance to next junction
	ID next_guide_node_id = m_extendedPath.back().cur_node_id;
	ID next_guide_edge_id = 0;
	for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
	{
		m_extendedPath[i].next_guide_node_id = next_guide_node_id;
		m_extendedPath[i].next_guide_edge_id = next_guide_edge_id;

		if (m_extendedPath[i].is_junction)
		{
			next_guide_node_id = m_extendedPath[i].cur_node_id;
			next_guide_edge_id = m_extendedPath[i].cur_edge_id;
		}
	}

	m_guide_idx = 0;
	//setInitialGuide();

	return true;

}


bool GuidanceManager::setInitialGuide()
{
	Guidance guide;

	//update GuideStatus
	guide.guide_status = GuideStatus::GUIDE_INITIAL;

	//update moving status
	guide.moving_status = MoveStatus::ON_EDGE;
	ExtendedPathElement curEP = getCurExtendedPath(0);
	ID cureid = curEP.cur_edge_id;
	ID targetnid = curEP.next_node_id;

	if (m_confidence > 0.5)	//when localizer is confident(used when path replanning)
	{
		//current robot's pose
		ID curnid = m_curpose.node_id;
		Node* curNode = m_map.findNode(curnid);
		cureid = curNode->edge_ids[m_curpose.edge_idx];
		Edge* curEdge = m_map.findEdge(cureid);
		ID nextnid = (curEdge->node_id1 == curnid) ? curEdge->node_id2 : curEdge->node_id1;
		Node* nextNode = m_map.findNode(nextnid);

		if (isNodeInPath(nextnid)) //already on right direction
		{
			targetnid = nextnid;
		}
		else //go to the first node
		{
			targetnid = curEP.cur_node_id;

			//check angle difference
			double cur_global_angle = atan2((nextNode->lat - curNode->lat), (nextNode->lon - curNode->lon));
			int cur_global_deg = (int)(cur_global_angle / PI * 180)	+ m_cur_head_degree;
			double to_global_angle = atan2((curNode->lat - nextNode->lat), (curNode->lon - nextNode->lon));
			int to_global_deg = (int)(to_global_angle / PI * 180);

			//derive turn angle
			int turn_degree = to_global_deg - cur_global_deg;

			//check degree
			if (!isForward(turn_degree))	//if TURN exists in past node
			{
				guide.actions.push_back(
					setActionTurn(curnid, cureid, turn_degree));

				//modify remain distance
				m_rmdistance = m_curpose.dist;
			}
		}
	}

	guide.actions.push_back(setActionGo(targetnid, cureid, 0));

	//update heading_node
	guide.heading_node_id = targetnid;

	//update distance_to_remain
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, MoveStatus::ON_EDGE);

	m_curguidance = guide;

	return true;
}


/** @brief setActionTurn reguire nodeId to find nodetype 
 *	to decide motion at the node
*/
GuidanceManager::Action GuidanceManager::setActionTurn(ID nid_cur, ID eid_cur, int degree_cur)
{
	Node* node = m_map.findNode(nid_cur);
	Edge* edge = m_map.findEdge(eid_cur);
	if (node == nullptr || edge == nullptr)
	{
		printf("[Error] GuidanceManager::setActionTurn\n");
		return GuidanceManager::Action();
	}
	Motion cmd = getMotion(node->type, edge->type, degree_cur);
	Mode mode = getMode(edge->type);
	Action result(cmd, node->type, edge->type, degree_cur, mode);

	return result;
}


GuidanceManager::Action GuidanceManager::setActionGo(ID nid_next, ID eid_cur, int degree)
{
	Node* node = m_map.findNode(nid_next);
	Edge* edge = m_map.findEdge(eid_cur);
	if (node == nullptr || edge == nullptr)
	{
		printf("[Error] GuidanceManager::setActionGo - No node or edge\n");
		return GuidanceManager::Action();
	}
	Motion cmd = getMotion(node->type, edge->type, degree);
	Mode mode = getMode(edge->type);

	Action result(cmd, node->type, edge->type, degree, mode);
	return result;
}

/** @brief getMotion require nodetype and edgetype connected to the node
 *	 to decide motion at the node
*/
GuidanceManager::Motion GuidanceManager::getMotion(int ntype, int etype, int degree)
{
	GuidanceManager::Motion motion;
	std::string rotation;

	if (ntype == Node::NODE_JUNCTION)
	{
		if (degree >= -45 && degree <= 45)	//"FORWARD"
		{
			if (etype == Edge::EDGE_CROSSWALK)
				motion = GuidanceManager::Motion::CROSS_FORWARD;
			else
				motion = GuidanceManager::Motion::GO_FORWARD;
		}
		else if (degree > 45 && degree <= 135)	//"LEFT"
		{
			if (etype == Edge::EDGE_CROSSWALK)
				motion = GuidanceManager::Motion::CROSS_LEFT;
			else
				motion = GuidanceManager::Motion::TURN_LEFT;
		}
		else if (degree < -45 && degree >= -135)	//"RIGHT"
		{
			if (etype == Edge::EDGE_CROSSWALK)
				motion = GuidanceManager::Motion::CROSS_RIGHT;
			else
				motion = GuidanceManager::Motion::TURN_RIGHT;
		}
		else //"BACK"
			motion = GuidanceManager::Motion::TURN_BACK;
	}
	else if (ntype == Node::NODE_DOOR)
	{
		if (degree >= -45 && degree <= 45)	//"FORWARD"
		{
			motion = GuidanceManager::Motion::ENTER_FORWARD;
		}
		else if (degree > 45 && degree <= 135)	//"LEFT"
		{
			motion = GuidanceManager::Motion::ENTER_LEFT;
		}
		else if (degree < -45 && degree >= -135)	//"RIGHT"
		{
			motion = GuidanceManager::Motion::ENTER_RIGHT;
		}
		else //"BACK"
			motion = GuidanceManager::Motion::TURN_BACK;
	}
	else
	{
		motion = GuidanceManager::Motion::GO_FORWARD;
	}

	return motion;
}


bool GuidanceManager::update(TopometricPose pose, double conf)
{
	//validate parameters
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::updateGuidance - Empty pose\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		return false;
	}

	if (setGuideStatus(pose, conf))
		if (applyPose(pose))
			if (setGuidanceWithGuideStatus())
				return true;

	setEmptyGuide();
	return false;
}


/**applyPose updates pose related variables.
	(m_mvstatus, m_guide_idx, m_distance)
*/
bool GuidanceManager::applyPose(TopometricPose  pose)
{
	m_curpose = pose;
	//validate parameter
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::applyPose - Empty pose!\n");
		m_mvstatus = MoveStatus::STOP_WAIT;
		return false;
	}

	//validate Current robot location
	ID nodeid = pose.node_id;
	Node* curnode = m_map.findNode(nodeid);
	if (curnode == nullptr)
	{
		printf("[Error] GuidanceManager::applyPose - The node is out-of-map!\n");
		m_mvstatus = MoveStatus::ON_EDGE;
		return false;
	}

	//update m_guide_idx
	int gidx = getGuideIdxFromPose(pose);
	if (gidx == -1)
	{
		printf("[Error] GuidanceManager::applyPose - The node is out-of-path!\n");
		m_mvstatus = MoveStatus::ON_EDGE;
		return false;
	}
	else if (gidx != m_guide_idx)//if new node appears
	{
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_guide_idx = gidx;
	}

	//check remain distance
	ExtendedPathElement curEP = getCurExtendedPath(m_guide_idx);
	ID eid = curnode->edge_ids[pose.edge_idx];
	Edge* curedge = m_map.findEdge(eid);
	double edgedist = curedge->length;
	double pastdist = pose.dist;
	bool junctionGuide = true;
	if (junctionGuide)
	{
		edgedist = curEP.remain_distance_to_next_junction;
	}
	m_rmdistance = edgedist - pastdist;

	//check heading
	m_cur_head_degree = (int) (pose.head/PI*180);

	/**Check progress.
	m_edge_progress: 0.0 ~ 1.0
	m_edge_progress indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/

	//Check edge following status
	if (pastdist < 1.0)
		m_mvstatus = MoveStatus::ON_NODE;
	else if (m_rmdistance < m_approachingThreshold)
		m_mvstatus = MoveStatus::APPROACHING_NODE;
	else
		m_mvstatus = MoveStatus::ON_EDGE;

	return true;
}

bool GuidanceManager::setGuideStatus(TopometricPose pose, double conf)
{
	m_confidence = conf;
	//validate parameters
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::setGuideStatus - Empty pose\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		return false;
	}

	if (m_extendedPath.size() < 1)
	{
		//printf("[Error] GuidanceManager::setGuideStatus - Empty path\n");
		m_gstatus = GuideStatus::GUIDE_NOPATH;
		//setEmptyGuide();
		return false;
	}

	ID nodeid = pose.node_id; //Current robot location
	//finishing condition
	if (nodeid == m_path.pts.back().node_id)
	{
		m_gstatus = GuideStatus::GUIDE_ARRIVED;
		return true;
	}

	Node* curnode = m_map.findNode(nodeid);
	ID edgeid = curnode->edge_ids[pose.edge_idx];
	if (isNodeInPath(nodeid) > 0)
	//if (isNodeEdgeInExtPath(nodeid, edgeid))
	{//as long as nodeid exists on path, everything is ok

		if (m_guide_idx == 0)
		{
			m_gstatus = GuideStatus::GUIDE_INITIAL;
			return true;
		}
		oop_start = 0;
		m_gstatus = GuideStatus::GUIDE_NORMAL;
		return true;
	}
	else
	{
		if (conf >= 0.1) //out-of-path 
		{//robot is in another path, for sure.
			if (oop_start == 0)	//start timer
				oop_start = time(NULL);
			oop_end = time(NULL);
			double diff_t = difftime(oop_end, oop_start);

			//timer is already running
			if (diff_t > 5.0)	//after 5 seconds
			{
				printf("The node(%zu) is out-of-path!\n", nodeid);
				m_gstatus = GuideStatus::GUIDE_OOP;
				return false;

			}
			printf("The node(%zu) is out-of-path detected!\n", nodeid);
			m_gstatus = GuideStatus::GUIDE_OOP_DETECT;
			return false;
		}
		else
		{//Lost, out-of-map (confidence is low or conf == -1)
			//localizer does not know where the robot is.
			//It needs to go back by recovery mode
			printf("Now we are lost! Unknown node: (%zu)\n", nodeid);
			m_gstatus = GuideStatus::GUIDE_LOST;
			return false;
		}
	}
	printf("[Error] GuidanceManager::updateGuidance - Unknown GuideStatus\n");
	m_gstatus = GuideStatus::GUIDE_UNKNOWN;
	return false;
}


bool GuidanceManager::setGuidanceWithGuideStatus()
{
	switch (m_gstatus)
	{
	case GuideStatus::GUIDE_INITIAL:
	{
		setInitialGuide();
		break;
	}
	case GuideStatus::GUIDE_NORMAL:
	{
		setNormalGuide();
		break;
	}
	case GuideStatus::GUIDE_ARRIVAL:
	{
		setArrivalGuide();
		break;
	}
	case GuideStatus::GUIDE_ARRIVED:
	{
		setArrivalGuide();
		break;
	}
	case GuideStatus::GUIDE_OOP_DETECT:
	{
		setNormalGuide();
		break;
	}
	case GuideStatus::GUIDE_OOP:
	{
		//setOOPGuide();
		break;
	}
	// case GuideStatus::GUIDE_LOST:
	// 	break;
	// case GuideStatus::GUIDE_RECOVERY:
	// 	break;
	// case GuideStatus::GUIDE_EXPLORATION:
	// 	break;
	// case GuideStatus::GUIDE_OPTIMAL_VIEW:
	// 	break;
	default:
		setEmptyGuide();
		break;
	}

	return true;
}

GuidanceManager::ExtendedPathElement GuidanceManager::getCurExtendedPath(int idx)
{
	if (idx < 0 || idx >= (int)m_extendedPath.size())
	{
		printf("[ERROR] GuidanceManager::getCurExtendedPath: idx = %d\n", idx);
		return GuidanceManager::ExtendedPathElement();
	}
	return m_extendedPath[idx];
}


bool GuidanceManager::setNormalGuide()
{
	if (m_guide_idx >= m_extendedPath.size() - 1)
	{
		printf("[Error] GuidanceManager::setNormalGuide() - normal guid cannot be called for last path node\n");
		return false;
	}

	Guidance guide;
	ExtendedPathElement curEP = getCurExtendedPath(m_guide_idx);
	curEP.next_guide_node_id;
	ExtendedPathElement nextEP = getCurExtendedPath(m_guide_idx + 1);
	//get angle
	int cur_angle = curEP.cur_degree - m_cur_head_degree;
	printf("cur_angle: %d, ", cur_angle);
	int next_angle = nextEP.cur_degree;
	printf("next_angle: %d \n", next_angle);

	//update dgstatus
	guide.guide_status = m_gstatus;

	//update moving status
	guide.moving_status = m_mvstatus;

	//set actions
	switch (m_mvstatus)
	{
	case MoveStatus::ON_NODE: //(TURN) - GO
	{
		//if TURN exists on past node, add former turn on current node//from last guide
		if (!isForward(curEP.cur_degree))	//if TURN exists in past node
		{
		//	int turnDeg = curEP.cur_degree;
			int turnDeg = curEP.cur_degree - m_cur_head_degree;	//if ON_NODE, turn remain degree
			guide.actions.push_back(
				setActionTurn(curEP.cur_node_id, curEP.cur_edge_id, turnDeg));
		}

		//add GO on current node
		guide.actions.push_back(setActionGo(curEP.next_node_id, curEP.cur_edge_id, 0));

		break;
	}
	case MoveStatus::ON_EDGE://maintain current guide, until next Node
	{	//only forward action is displayed on edge status
		guide.actions.push_back(setActionGo(curEP.next_node_id, curEP.cur_edge_id, 0));
		break;
	}
	case MoveStatus::APPROACHING_NODE://After 000m, (TURN) - GO, prepare next Node action
	{
		//GO_FORWARD, add next node action
		if (nextEP.cur_edge_id == 0 || nextEP.next_node_id == 0)	//heading last node
		{
			guide.actions.push_back(setActionGo(curEP.next_node_id, curEP.cur_edge_id, 0));
		}
		else
		{
			if (!isForward(nextEP.cur_degree))//if TURN exists on next node,
			{
				int turnDeg = nextEP.cur_degree;	//if APPROACHING_NODE, turn next degree
				//int turnDeg = nextEP.cur_degree - m_cur_head_angle;
				guide.actions.push_back(
					setActionTurn(nextEP.cur_node_id, nextEP.cur_edge_id, turnDeg));
			}
			guide.actions.push_back(setActionGo(nextEP.next_node_id, nextEP.cur_edge_id, 0));
		}
		break;
	}
	case MoveStatus::STOP_WAIT:
	{
		guide.actions[0].cmd = Motion::STOP;
		guide.msg = "No pose info";
		m_curguidance = guide;
		return false;
	}
	default:
		break;
	}

	//update heading_node
	guide.heading_node_id = curEP.next_node_id;

	//make string msg
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, m_mvstatus);

	m_curguidance = guide;

	return true;
}


bool GuidanceManager::setArrivalGuide()
{
	Guidance guide;
	ExtendedPathElement lastguide = m_extendedPath.back();
	Node* dest = m_map.findNode(lastguide.cur_node_id);

	//update dgstatus
	guide.guide_status = m_gstatus;

	//update moving status
	guide.moving_status = m_mvstatus;

	//set actions
	
	//if last turn exists,
	if (!isForward(m_finalTurn))
	{
		Motion cmd = getMotion(dest->type, Edge::EDGE_SIDEWALK, m_finalTurn);
		Mode mode = getMode(Edge::EDGE_SIDEWALK);
		Action action(cmd, dest->type, Edge::EDGE_SIDEWALK, m_finalTurn, mode);
		guide.actions.push_back(action);
		guide.msg = getStringTurn(action, dest->type) + " and ";
	}

	guide.guide_status = GuideStatus::GUIDE_ARRIVED;
	guide.actions.push_back(Action(Motion::STOP, Node::NODE_BASIC, Edge::EDGE_SIDEWALK, 0, Mode::MOVE_NORMAL));
	guide.distance_to_remain = 0;
	guide.msg = guide.msg + "[GUIDANCE] Arrived!";
	m_curguidance = guide;
	m_extendedPath.clear();


	//update heading_node
	guide.heading_node_id = lastguide.cur_node_id;

	//make string msg
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, m_mvstatus);

	m_curguidance = guide;

	return true;
}


//bool GuidanceManager::setArrivalGuide()
//{
//	ExtendedPathElement lastguide = m_extendedPath.back();
//	Node* dest = m_map.findNode(lastguide.cur_node_id);
//	if (dest == nullptr)
//	{
//		setEmptyGuide();
//		return false;
//	}
//
//	Guidance guide;
//	//if last turn exists,
//	if (!isForward(m_finalTurn))
//	{
//		Motion cmd = getMotion(dest->type, Edge::EDGE_SIDEWALK, m_finalTurn);
//		Mode mode = getMode(Edge::EDGE_SIDEWALK);
//		Action action(cmd, Edge::EDGE_SIDEWALK, m_finalTurn, mode);
//		guide.actions.push_back(action);
//		guide.msg = getStringTurn(action, dest->type) + " and ";
//	}
//
//	guide.guide_status = GuideStatus::GUIDE_ARRIVED;
//	guide.actions.push_back(Action(Motion::STOP, Edge::EDGE_SIDEWALK, 0, Mode::MOVE_NORMAL));
//	guide.distance_to_remain = 0;
//	guide.msg = guide.msg + "[GUIDANCE] Arrived!";
//	m_curguidance = guide;
//	m_extendedPath.clear();
//
//	return true;
//}


bool GuidanceManager::setEmptyGuide()
{
	Guidance guide;
	//guide.guide_status = GuideStatus::GUIDE_NOPATH;
	//guide.actions.push_back(Action(Motion::STOP, Edge::EDGE_SIDEWALK, 0, Mode::MOVE_NORMAL));
	guide.distance_to_remain = 0;
	guide.msg = "";

	m_curguidance = guide;

	return true;
}


std::string GuidanceManager::getStringForward(Action act, int ntype, ID nid, double d)
{
	std::string result;

	//check parameter
	if ((int)act.cmd < 0 || act.cmd >= Motion::TYPE_NUM || (int) act.edge_type < 0 || act.edge_type >= Edge::TYPE_NUM)
	{
		printf("[Error] GuidanceManager::getStringForward() - wrong Action!\n");
		return result;
	}

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string str_act = motion + " on " + edge;

	std::string nodetype = m_nodes[ntype];

	std::string nodeid = std::to_string(nid);
	std::string distance = std::to_string(d).substr(0,4);

	std::string act_add = " about " + distance + "m" + " until next " + nodetype + "(Node ID : " + nodeid + ")";
	result = str_act + act_add;
	return result;
}

std::string GuidanceManager::getStringTurn(Action act, int ntype)
{
	std::string result;

	//check parameter
	if ((int)act.cmd < 0 || act.cmd >= Motion::TYPE_NUM || (int)act.edge_type < 0 || act.edge_type >= Edge::TYPE_NUM)
	{
		printf("[Error] GuidanceManager::getStringTurn() - wrong Action!\n");
		return result;
	}
	if (ntype < 0 || ntype >= Node::TYPE_NUM)
	{
		printf("[Error] GuidanceManager::getStringTurn() - wrong Node!\n");
		return result;
	}

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);
	std::string str_act = motion + " for " + degree + " degree";
	std::string nodetype = m_nodes[ntype];
	std::string act_add = " on " + nodetype;
	result = str_act + act_add;
	return result;
}

std::string GuidanceManager::getStringTurnDist(Action act, int ntype, double dist)
{
	std::string result;

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);

	if (act.cmd == Motion::TURN_BACK)
	{
		result = motion + " for " + degree + " degree";
	}
	else
	{
		std::string distance = std::to_string(dist);
		std::string str_act = "After " + distance + "m " + motion + " for " + degree + " degree";
		std::string nodetype = m_nodes[ntype];
		std::string act_add = " on " + nodetype;
		result = str_act + act_add;
	}
	
	return result;

}

std::string GuidanceManager::getStringGuidance(Guidance guidance, MoveStatus status)
{
	std::string result, str_first;
	std::vector<std::string> str;
	std::vector<Action> actions = guidance.actions;

	int cnt = 0;
	for (size_t i = 0; i < actions.size(); i++)
	{
		if (isForward(actions[i].cmd))
		{
			str_first = getStringForward(actions[i], actions[i].node_type,
				guidance.heading_node_id, guidance.distance_to_remain);
		}
		else
		{
			if (m_mvstatus == MoveStatus::ON_NODE)
				str_first = getStringTurn(actions[i], actions[i].node_type);
			else
				str_first = getStringTurnDist(actions[i], actions[i].node_type, guidance.distance_to_remain);
		}
		str.push_back(str_first);
	}

	//result = "[Guide] " + str[0];

	result = "[Guide] [" + m_movestates[(int)status] + "]" + str[0];
	if (actions.size() >= 2)	//only 2 steps are shown in msg
		result = result + " and " + str[1];

	return result;
}

bool GuidanceManager::isNodeInPath(ID nodeid)
{
	for (size_t i = 0; i < m_path.pts.size(); i++)
	{
		if (nodeid == m_path.pts[i].node_id)
			return true;
	}
	return false;
}

bool GuidanceManager::isNodeEdgeInExtPath(ID nodeid, ID edgeid)
{
	for (size_t i = 0; i < m_extendedPath.size(); i++)
	{
		if (nodeid == m_extendedPath[i].cur_node_id)
			if (edgeid == m_extendedPath[i].cur_edge_id)
				return true;
	}
 	return false;
}

int GuidanceManager::getGuideIdxFromPose(TopometricPose pose)
{
	ID curnodei = pose.node_id;
	for (size_t i = 0; i < m_extendedPath.size(); i++)
	{
		if (m_extendedPath[i].cur_node_id == curnodei)
		{
			return (int)i;
		}
	}
	return -1;
}


bool GuidanceManager::applyPoseGPS(LatLon gps)
{
	if (gps.lat <= 0 || gps.lon <= 0)
	{
		printf("[Error] GuidanceManager::applyPoseGPS]No GPS info!\n");
		return false;
	}

	m_latlon = gps;
	return true;
};

void GuidanceManager::makeLostValue(double prevconf, double curconf)
{
	double lowerlimit = 0.4;
	double upperlimit = 0.85;
	double middlevalue = 0.7;
	int weight = 150;
	int smallweight = 75;

	if (curconf <= lowerlimit)
		m_lostvalue = 100.0;
	else if (curconf >= upperlimit)
		m_lostvalue = 0.0;
	else if (curconf < middlevalue)
	{
		if (middlevalue <= prevconf)
			m_lostvalue = m_lostvalue + ((prevconf - curconf) * weight);
		else
		{
			if (curconf < prevconf)
				m_lostvalue = m_lostvalue + ((middlevalue - curconf) * weight);
			else
				m_lostvalue = m_lostvalue + ((middlevalue - curconf) * smallweight);
		}
	}
	else
	{
		if (prevconf <= middlevalue)
			m_lostvalue = m_lostvalue - ((curconf - prevconf) * weight);
		else
		{
			if (curconf < prevconf)
				m_lostvalue = m_lostvalue - ((curconf - middlevalue) * smallweight);
			else
				m_lostvalue = m_lostvalue - ((curconf - middlevalue) * weight);
		}
	}

	if (m_lostvalue > 100.0)
		m_lostvalue = 100.0;
	else if (m_lostvalue < 0.0)
		m_lostvalue = 0.0;

	m_prevconf = curconf;
}

// bool updateActiveNav(cv::Mat image, GuidanceManager::Guidance guidance)
// {
//     dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
//     extern dg::ActiveNavigation m_active_nav;
// 	m_active_nav.apply(image, guidance, t1);	
// }


// test code
//GuidanceManager guider;
//int t1 = guider.getDegree(0.0, 0.0, 1.0, 0.0, 1.0, 1.0);
//printf("t1: %d\n", t1);
//int t2 = guider.getDegree(0.0, 0.0, 1.0, 0.0, 1.0, -1.0);
//printf("t2: %d\n", t2);
//int t3 = guider.getDegree(0.0, 0.0, -1.0, 0.0, -1.0, 1.0);
//printf("t3: %d\n", t3);
//int t4 = guider.getDegree(0.0, 0.0, -1.0, 0.0, -1.0, -1.0);
//printf("t4: %d\n", t4);
//int GuidanceManager::getDegree(double x1, double y1, double x2, double y2, double x3, double y3)
//{
//	double v1x = x2 - x1;
//	double v1y = y2 - y1;
//	double v2x = x3 - x2;
//	double v2y = y3 - y2;
//
//	if (!(v1x * v1x + v1y * v1y) || !(v2x * v2x + v2y * v2y))
//	{
//		int result = 0;
//		return result;
//	}
//
//	double sign = asin((v1x * v2y - v1y * v2x) / (sqrt(v1x * v1x + v1y * v1y) * sqrt(v2x * v2x + v2y * v2y)));
//	double rad = acos((v1x * v2x + v1y * v2y) / (sqrt(v1x * v1x + v1y * v1y) * sqrt(v2x * v2x + v2y * v2y)));
//
//	double rad2deg = rad / 3.14 * 180.0;
//	//if (sign < 0.f) rad2deg = 360.f - rad2deg;	//this is for 0~360
//	if (sign < 0.f) rad2deg = - rad2deg;	//this is for -180~180
//
//
//	int result = (int)rad2deg;
//
//	return result;
//
//}
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

	double rad2deg = rad / PI * 180.0;
	if (sign < 0.f) rad2deg = -rad2deg;	//this is for -180~180

	int result = (int)rad2deg;

	return result;

}

bool GuidanceManager::validatePath(Path& path, Map& map)
{
	for (size_t i = 0; i < path.pts.size() - 2; i++)
	{
		Node* curnode = map.findNode(path.pts[i].node_id);
		if (curnode == nullptr)
		{
			printf("No Node-%zu found on map!\n", path.pts[i].node_id);
			return false;
		}
		Edge* curedge = map.findEdge(path.pts[i].edge_id);
		if (curedge == nullptr)
		{
			printf("No Edge-%zu found on map!\n", path.pts[i].edge_id);
			return false;
		}
	}
	return true;
}

/** obsolete
bool GuidanceManager::setOOPGuide()
{	//need new map and generate path from current pose
	//	take original goal and change start to current pose
	Guidance guide;
	double start_lat, start_lon, dest_lat, dest_lon;

	LatLon curGPS = getPoseGPS();
	start_lat = curGPS.lat;
	start_lon = curGPS.lon;
	Node* dest = m_map.findNode(m_path.pts.back().node_id);
	dest_lat = dest->lat;
	dest_lon = dest->lon;
	if (!regeneratePath(start_lat, start_lon, dest_lat, dest_lon))
	{
		setEmptyGuide();
		return false;
	}
	setNormalGuide();
	return true;
}

bool GuidanceManager::regeneratePath(double start_lat, double start_lon, double dest_lat, double dest_lon)
{
	//replace to new path
	MapManager map_manager;
	Path path;
	map_manager.getPath(start_lat, start_lon, dest_lat, dest_lon, path);

	Map map = map_manager.getMap();
	m_map.set_union(map);
	initiateNewGuidance(path, m_map);

	//restart with index 0
	if (!buildGuides()) return false;
	return true;
}

Node* dg::GuidanceManager::findNodeFromPath(ID nodeid)
{
	for (size_t i = 0; i < m_path.pts.size(); i++)
	{
		if (m_path.pts[i].node->id == nodeid)
		{
			return m_path.pts[i].node;
		}
	}
	return nullptr;
}

Edge* dg::GuidanceManager::findEdgeFromPath(ID edgeid)
{
	for (size_t i = 0; i < m_path.pts.size(); i++)
	{
		if (m_path.pts[i].edge->id == edgeid)
		{
			return m_path.pts[i].edge;
		}
	}
	return nullptr;
}

*/