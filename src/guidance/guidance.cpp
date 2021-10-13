#include "guidance.hpp"
#include "dg_map_manager.hpp"
#include "utils/opencx.hpp"

using namespace dg;

bool GuidanceManager::initialize(SharedInterface* shared)
{
	m_shared = shared;
	return (m_shared != nullptr);
}

bool GuidanceManager::initiateNewGuidance()
{
	return buildGuides();
}

bool GuidanceManager::buildGuides()
{
	// check map and path
	Map* map = getMap();
	Path* path = getPath();
	if (map == nullptr || map->isEmpty())
	{
		printf("[Error] GuidanceManager::buildGuides - Empty Map\n");
		return false;
	}
	if (path == nullptr || path->empty())
	{
		printf("[Error] GuidanceManager::buildGuides - Empty path\n");
		return false;
	}
	if (!validatePath(*path, *map))
	{
		printf("[Error] GuidanceManager::buildGuides - Path is not in the map!\n");
		return false;
	}

	// clear old guidance path
	m_extendedPath.clear();

	// build new guidance path (exclude first and last path element: temporal start and destination node)
	for (int i = 1; i < (int)path->pts.size() - 1; i++)
	{
		ID curnid = path->pts[i].node_id;
		ID cureid = path->pts[i].edge_id;
		ID nextnid = path->pts[i + 1].node_id;
		ID nexteid = path->pts[i + 1].edge_id;
		Node* curNode = getMap()->getNode(curnid);
		if (curNode == nullptr)
		{
			printf("[Error] GuidanceManager::buildGuides - Node-%zd is not in the map!\n", curnid);
			return false;
		}

		int angle = 0;
		if (i > 0)
		{
			angle = getDegree(path->pts[i - 1], path->pts[i], path->pts[i + 1]);
		}
		ExtendedPathElement tmppath(curnid, cureid, nextnid, nexteid, angle, path->pts[i].x, path->pts[i].y);

		if (curNode && (curNode->type == Node::NODE_JUNCTION || curNode->type == Node::NODE_DOOR))
		{
			tmppath.is_junction = true;
		}

		m_extendedPath.push_back(tmppath);
	}

	//add last node
	m_finalTurn = 0;
	m_extendedPath.back().cur_degree = m_finalTurn;

	// update remain distance to next junction
	double d_accumulated = 0;
	for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
	{
		ID eid = m_extendedPath[i].cur_edge_id;
		Edge* edge = getMap()->getEdge(eid);
		if (edge) d_accumulated += edge->length;
		else d_accumulated += norm(m_extendedPath[i + 1] - m_extendedPath[i]);

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
		Edge* edge = getMap()->getEdge(eid);
		if (edge) d_accumulated += edge->length;
		else d_accumulated += norm(m_extendedPath[i] - m_extendedPath[i - 1]);

		m_extendedPath[i].past_distance_from_prev_junction = d_accumulated;

		if (m_extendedPath[i].is_junction)
		{
			d_accumulated = 0;
		}
	}

	// update remain distance to next junction
	ID next_guide_node_id = m_extendedPath.back().cur_node_id;
	ID next_guide_edge_id = m_extendedPath.back().cur_edge_id;
	for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
	{
		m_extendedPath[i].next_guide_node_id = next_guide_node_id;
		m_extendedPath[i].next_guide_edge_id = next_guide_edge_id;

		if (next_guide_node_id == 0 || m_extendedPath[i].is_junction)
		{
			next_guide_node_id = m_extendedPath[i].cur_node_id;
			next_guide_edge_id = m_extendedPath[i].cur_edge_id;
		}
	}

	m_guide_idx = 0;

	// for (size_t i = 0; i < m_extendedPath.size(); i++)
	// {
	// 	printf("[%d] Node id:%zu, Deg: %d \n", i, m_extendedPath[i].cur_node_id, m_extendedPath[i].cur_degree);
	// }
	
	return true;

}

// bool GuidanceManager::setTunBackGuide()
// {
// 	Guidance guide;

// 	//update GuideStatus
// 	guide.guide_status = m_gstatus;
	
// 	//update moving status
// 	guide.moving_status = m_mvstatus;

// 	ID curnid = m_curpose.node_id;
// 	Node* curnode = getMap()->getNode(curnid);
// 	ID cureid = curnode->edge_ids[m_curpose.edge_idx];

// 	guide.actions.push_back(
// 		setActionTurn(curnid, cureid, 180));
	
// 	//update distance_to_remain
// 	m_rmdistance = m_curpose.dist;
// 	guide.distance_to_remain = m_rmdistance;
		
// 	//update heading_node
// 	guide.heading_node_id = curnid;

// 	//make guidance string
// 	guide.msg = getStringGuidance(guide, m_mvstatus);

// 	m_curguidance = guide;

// 	return true;
// }

bool GuidanceManager::setInitialGuide()
{
	Guidance guide;

	//update GuideStatus
	guide.guide_status = m_gstatus;
	
	//update moving status
	guide.moving_status = m_mvstatus;

	ExtendedPathElement firstEP = getCurExtendedPath(0);
	bool bTurn = false;
	if (m_confidence > 0.5)	//when localizer is confident(used when path replanning)
	{
		//current robot's pose
		ID curnid = m_curpose.node_id;
		Node* curNode = getMap()->getNode(curnid);
		ID cureid = curNode->edge_ids[m_curpose.edge_idx];
		Edge* curEdge = getMap()->getEdge(cureid);
		ID nextnid = (curEdge->node_id1 == curnid) ? curEdge->node_id2 : curEdge->node_id1;

		//if wrong direction
		// printf("[applyPose] nextnid: %zu, curEP.next_node_id: %zu!\n", nextnid, curEP.next_node_id);

		if (!isNodeInPath(nextnid) || (isEdgeInPath(cureid) && nextnid == firstEP.cur_node_id))
		{			
			printf("[setInitialGuide]wrong direction! cureid: %zu, nextnid: %zu, firstEP.cur_node_id: %zu\n", cureid, nextnid, firstEP.cur_node_id);
			guide.actions.push_back(
				setActionTurn(curnid, cureid, 180));

			bTurn = true;
			//modify remain distance
			m_rmdistance = m_curpose.dist;
		}
		
		//find next path
		ExtendedPathElement nextEP;
		if (isEdgeInPath(cureid))
			nextEP = getCurExtendedPath(1);
		else
			nextEP = firstEP;

		if(m_mvstatus != MovingStatus::ON_EDGE)//After 000m, (TURN) - GO, prepare next Node action
		{			
			if (!isForward(nextEP.cur_degree))//if TURN exists on next node,
			{
				printf("nextEP.cur_degree: %d \n", nextEP.cur_degree);
				guide.actions.push_back(
					setActionTurn(nextEP.cur_node_id, nextEP.cur_edge_id, nextEP.cur_degree));
			}
			guide.actions.push_back(setActionGo(nextEP.next_node_id, nextEP.cur_edge_id, 0));
		}
		else
		{			
			guide.actions.push_back(setActionGo(nextEP.cur_node_id, cureid, 0));
		}		

	}
		
	//update heading_node
	guide.heading_node_id = firstEP.next_node_id;

	//update distance_to_remain
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, m_mvstatus);

	m_curguidance = guide;

	return true;
}

/** @brief setActionTurn reguire nodeId to find nodetype 
 *	to decide motion at the node
*/
GuidanceManager::Action GuidanceManager::setActionTurn(ID nid_cur, ID eid_cur, int degree_cur)
{
	Node* node = getMap()->getNode(nid_cur);
	Edge* edge = getMap()->getEdge(eid_cur);
	if (node == nullptr || edge == nullptr)
	{
		printf("[Error] GuidanceManager::setActionTurn\n");
		return GuidanceManager::Action();
	}
	Motion cmd = getMotion(node->type, edge->type, degree_cur);
	MotionMode mode = getMode(edge->type);
	Action result(cmd, node->type, edge->type, degree_cur, mode);

	return result;
}


GuidanceManager::Action GuidanceManager::setActionGo(ID nid_next, ID eid_cur, int degree)
{
	Node* node = getMap()->getNode(nid_next);
	Edge* edge = getMap()->getEdge(eid_cur);
	if (node == nullptr || edge == nullptr)
	{
		printf("[Error] GuidanceManager::setActionGo - No node or edge\n");
		return GuidanceManager::Action();
	}
	Motion cmd = getMotion(node->type, edge->type, degree);
	MotionMode mode = getMode(edge->type);

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
		if (degree >= -30 && degree <= 30)	//"FORWARD"
		{
			if (etype == Edge::EDGE_CROSSWALK)
				motion = GuidanceManager::Motion::CROSS_FORWARD;
			else
				motion = GuidanceManager::Motion::GO_FORWARD;
		}
		else if (degree > 30 && degree <= 150)	//"LEFT"
		{
			if (etype == Edge::EDGE_CROSSWALK)
				motion = GuidanceManager::Motion::CROSS_LEFT;
			else
				motion = GuidanceManager::Motion::TURN_LEFT;
		}
		else if (degree < -30 && degree >= -150)	//"RIGHT"
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
		if (degree >= -30 && degree <= 30)	//"FORWARD"
		{
			motion = GuidanceManager::Motion::ENTER_FORWARD;
		}
		else if (degree > 30&& degree <= 150)	//"LEFT"
		{
			motion = GuidanceManager::Motion::ENTER_LEFT;
		}
		else if (degree < -30 && degree >= -150)	//"RIGHT"
		{
			motion = GuidanceManager::Motion::ENTER_RIGHT;
		}
	}
	else
	{
		if (degree > 150 || degree < -150) //"BACK"
		{
			motion = GuidanceManager::Motion::TURN_BACK;
		}
		else
		{
			motion = GuidanceManager::Motion::GO_FORWARD;
		}
	}

	return motion;
}

bool GuidanceManager::update(TopometricPose pose)
{
    int guide_interval = 10; //m
	int gidx = getGuideIdxFromPose(pose);
	if (gidx != m_guide_idx)//if new node appears
	{
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_guide_idx = gidx;
	}
	ExtendedPathElement curEP = getCurExtendedPath(m_guide_idx);
	
	//check remain distance
	Node* curnode = getMap()->getNode(pose.node_id);
	ID cureid = curnode->edge_ids[pose.edge_idx];
	Edge* curedge = getMap()->getEdge(cureid);
	double passsed_dist = pose.dist;
	double edge_dist = curEP.remain_distance_to_next_junction;        
	double remain_dist = edge_dist - passsed_dist;
	m_rmdistance = remain_dist;
	int announce_dist = (int) remain_dist / guide_interval * guide_interval;
	printf("announce_dist: %d\n",announce_dist);	
	bool announce = false;
	//near junction
	printf("remain_dist: %.2f, %.2f, %.2f\n", remain_dist, m_uncertain_dist, passsed_dist);
	if (remain_dist <= m_uncertain_dist || passsed_dist <= m_uncertain_dist)
	{
		printf("edge_dist: %f, %f\n", edge_dist, 2*m_uncertain_dist);
		if(edge_dist <= 2*m_uncertain_dist)
		{
			printf("m_last_announce_dist: %d, %d, %d\n", m_last_announce_dist, announce_dist, announce);
			if((m_last_announce_dist != announce_dist) && !announce)
			{
				announce = true;
				setSimpleGuide();
				m_last_announce_dist = announce_dist;
			}
			else
				announce = false;
		}
		else
			announce = false;
	}	
	//on junction
	else if (passsed_dist < m_approachingThreshold || remain_dist < m_approachingThreshold)
	{
		printf("Too close to junction!\n");
		m_last_announce_dist = -1;
		announce = false;
	}
	//normal case
	else
	{		
		printf("announce_dist: %d, m_last_announce_dist: %d,\n", announce_dist, m_last_announce_dist);
		if(announce_dist != m_last_announce_dist)
		{
			announce = true;
			setSimpleGuide();
			printf("update announce_dist\n");
			m_last_announce_dist = announce_dist;
		}
		else
			announce = false;
	}
	m_curguidance.announce = announce;

	
    return true;
}
/*
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
*/


/**applyPose updates pose related variables.
	(m_mvstatus, m_guide_idx, m_distance)
*/
bool GuidanceManager::applyPose(TopometricPose  pose)
{
	m_curpose = pose;
	MovingStatus mvstatus = MovingStatus::ON_EDGE;
	//validate node id
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::applyPose - Empty pose!\n");
		m_mvstatus = MovingStatus::STOP_WAIT;
		return false;
	}

	//validate Current robot location 
	ID curnid = pose.node_id;
	Node* curnode = getMap()->getNode(curnid);
	if (curnode == nullptr)
	{
		printf("[Error] GuidanceManager::applyPose - curnode == nullptr!\n");
		m_mvstatus = MovingStatus::ON_EDGE;
		return false;
	}

	//update m_guide_idx
	int gidx = getGuideIdxFromPose(pose);
	if (gidx == -1)	//cannot find the curnids. initial or oop statu
	{// in initial state the pose may be 
		// if (m_gstatus == GuideStatus::GUIDE_INITIAL) //if initial state
		// {
		// 	m_mvstatus = MovingStatus::ON_EDGE;
		// }
		// else //if oop
		// {
		// 	printf("[Error] GuidanceManager::applyPose - gidx == -1!\n");
		// 	m_mvstatus = MovingStatus::ON_EDGE;
		// }
		m_mvstatus = MovingStatus::ON_EDGE;
		return true;		
	}
	else if (gidx != m_guide_idx)//if new node appears
	{
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_guide_idx = gidx;
	}

	ExtendedPathElement curEP = getCurExtendedPath(gidx);
	ID cureid = curnode->edge_ids[pose.edge_idx];
	Edge* curedge = getMap()->getEdge(cureid);

	//check remain distance
	double edgedist = curedge->length;
	double pastdist = pose.dist;
	if (m_juctionguide)
	{
		// printf("[applyPose] cureid: %zu, curEP.cur_edge_id: %zu!\n", cureid, curEP.cur_edge_id);
		if (cureid == curEP.cur_edge_id)
			edgedist = curEP.remain_distance_to_next_junction;
	}
	m_rmdistance = edgedist - pastdist;

	//check heading
	m_cur_head_degree = (int)cx::cvtRad2Deg(pose.head);

	/**Check progress.
	m_edge_progress: 0.0 ~ 1.0
	m_edge_progress indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/

	//Check edge following status
	if (pastdist < m_approachingThreshold)
		m_mvstatus = MovingStatus::ON_NODE;
	else if (m_rmdistance < m_approachingThreshold)
	{		
		if(curEP.cur_edge_id == cureid) //on initial stage, sometimes they are not in same edge
			m_mvstatus = MovingStatus::APPROACHING_NODE;

		//if the robot is on crosswalk, maintain current guide
		if (curedge->type == Edge::EDGE_CROSSWALK && m_rmdistance > curedge->length/2)
			m_mvstatus = MovingStatus::ON_EDGE;
		printf("mvstatus %d, curedge->type: %d, m_rmdistance: %.2f, curedge->length/2: %.2f\n",(int)m_mvstatus, (int) curedge->type, m_rmdistance, curedge->length/2);
	}
	else
		m_mvstatus = MovingStatus::ON_EDGE;
	// printf("m_mvstatus: %d\n", m_mvstatus);
	return true;
}

bool GuidanceManager::setGuideStatus(TopometricPose pose, double conf)
{
	m_confidence = conf;
	ID curNId = pose.node_id; //Current robot location
	
	//validate node id
	if (curNId == 0)
	{
		printf("[Error] GuidanceManager::setGuideStatus - Empty pose\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		return false;
	}

	//validate path
	if (m_extendedPath.size() < 1)
	{
		m_gstatus = GuideStatus::GUIDE_NOPATH;
		return false;
	}

	//finishing condition
	if (curNId == m_extendedPath.back().cur_node_id)
	{
		m_gstatus = GuideStatus::GUIDE_ARRIVED;
		m_arrival = true;
//		printf("[setGuideStatus]finishing m_gstatus: %d\n", m_gstatus);
		return true;
	}

	//after arrival, continuously moving.
	if (m_arrival && curNId != m_extendedPath.back().cur_node_id)
	{
		m_arrival = false;
		m_extendedPath.clear();
		m_gstatus = GuideStatus::GUIDE_NOPATH;
		return false;
	}

	//check initial status
	//current robot's pose
	ID curnid = pose.node_id;
	Node* curNode = getMap()->getNode(curnid);
	if (curNode == nullptr)
	{
		printf("[Error] GuidanceManager::setGuideStatus - curNode%zu == nullptr!\n", curnid);
		return false;
	}
	ID cureid = curNode->edge_ids[pose.edge_idx];
	Edge* curEdge = getMap()->getEdge(cureid);
	if (curEdge == nullptr)
	{
		printf("[Error] GuidanceManager::setGuideStatus - curEdge%zu == nullptr!\n", cureid);
		return false;
	}
	ID nextnid = (curEdge->node_id1 == curnid) ? curEdge->node_id2 : curEdge->node_id1;
	
   	// printf("[setGuideStatus]isNodeInPath(curnid) %s!\n", isNodeInPath(curnid) ? "T" : "F");
  	// printf("[setGuideStatus]isNodeInPath(nextnid) %s!\n", isNodeInPath(nextnid) ? "T" : "F");
	
	if (m_guide_idx == 0 && (isNodeInPath(curnid) || isNodeInPath(nextnid)))
	{
		m_gstatus = GuideStatus::GUIDE_INITIAL;
		// printf("[setGuideStatus]idx == 0 m_gstatus: %d\n", m_gstatus);
		return true;
	}

	Node* curnode = getMap()->getNode(curNId);
	ID edgeid = curnode->edge_ids[pose.edge_idx];
	if (isNodeInPath(curNId))
	{//as long as curNId exists on path, everything is ok
		oop_start = 0;
		m_gstatus = GuideStatus::GUIDE_NORMAL;
//		printf("[setGuideStatus] isNodeInPath>0 m_gstatus: %d\n", m_gstatus);
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
				printf("The node(%zu) is out-of-path!\n", curNId);
				m_gstatus = GuideStatus::GUIDE_OOP;
				return false;

			}
			printf("The node(%zu) is out-of-path detected!\n", curNId);
			m_gstatus = GuideStatus::GUIDE_OOP_DETECT;
			return false;
		}
		else
		{//Lost, out-of-map (confidence is low or conf == -1)
			//localizer does not know where the robot is.
			//It needs to go back by recovery mode
			printf("Now we are lost! Unknown node: (%zu)\n", curNId);
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
	case GuideStatus::GUIDE_NEAR_ARRIVAL:
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
	// case GuideStatus::GUIDE_TURNBACK:
	// {
	// 	setTunBackGuide();
	// 	break;
	// }
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

bool GuidanceManager::setSimpleGuide()
{
	Guidance guide;
	ExtendedPathElement curEP = getCurExtendedPath(m_guide_idx);
	ExtendedPathElement nextEP = getCurExtendedPath(m_guide_idx + 1);
	printf("Turn: %d\n", nextEP.cur_degree);
	if (!isForward(nextEP.cur_degree))//if TURN exists on next node,
	{
		int turnDeg = nextEP.cur_degree;	//if APPROACHING_NODE, turn next degree
		guide.actions.push_back(
			setActionTurn(nextEP.cur_node_id, nextEP.cur_edge_id, turnDeg));
	}
	else
		guide.actions.push_back(setActionGo(nextEP.next_node_id, nextEP.cur_edge_id, 0));

	//update heading_node
	guide.heading_node_id = curEP.next_node_id;

	//make string msg
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, m_mvstatus);

	m_curguidance = guide;

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
	ExtendedPathElement nextEP = getCurExtendedPath(m_guide_idx + 1);
	//get angle
	// int cur_angle = curEP.cur_degree - m_cur_head_degree;
	// printf("cur_angle: %d, ", cur_angle);
	// int next_angle = nextEP.cur_degree;
	// printf("next_angle: %d \n", next_angle);

	//update dgstatus
	guide.guide_status = m_gstatus;

	//update moving status
	guide.moving_status = m_mvstatus;

	//set actions
	// printf("[setNormalGuide] m_mvstatus: %d\n", m_mvstatus);
	switch (m_mvstatus)
	{
	case MovingStatus::ON_NODE: //(TURN) - GO
	{
		//if TURN exists on past node, add former turn on current node//from last guide
		if (!isForward(curEP.cur_degree))	//if TURN exists in past node
		{
			int turnDeg = curEP.cur_degree;
			// int turnDeg = curEP.cur_degree - m_cur_head_degree;	//if ON_NODE, turn remain degree
			guide.actions.push_back(
				setActionTurn(curEP.cur_node_id, curEP.cur_edge_id, turnDeg));
		}

		//add GO on current node
		guide.actions.push_back(setActionGo(curEP.next_node_id, curEP.cur_edge_id, 0));

		break;
	}
	case MovingStatus::ON_EDGE://maintain current guide, until next Node
	{	//only forward action is displayed on edge status

		if (!isForward(nextEP.cur_degree))//if TURN exists on next node,
		{
			int turnDeg = nextEP.cur_degree;	//if APPROACHING_NODE, turn next degree
			guide.actions.push_back(
				setActionTurn(nextEP.cur_node_id, nextEP.cur_edge_id, turnDeg));
		}
		else
			guide.actions.push_back(setActionGo(nextEP.next_node_id, nextEP.cur_edge_id, 0));
		break;
	}
	case MovingStatus::APPROACHING_NODE://After 000m, (TURN) - GO, prepare next Node action
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
	case MovingStatus::STOP_WAIT:
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


	//make announcement
	int cur_announce_dist = m_rmdistance;
	if (cur_announce_dist>10)
		cur_announce_dist = cur_announce_dist/10;
	if (m_mvstatus != MovingStatus::ON_EDGE || m_past_announce == cur_announce_dist)
	{
		guide.announce = 0;
		printf("guide.announce = 0\n");
	}
	else
	{
		guide.announce = 1;
		m_past_announce = cur_announce_dist;
		printf("guide.announce = 1\n");
		printf("%d\n", (int) m_rmdistance);
        std::string dist = std::to_string((int) m_rmdistance);
	}

	m_curguidance = guide;

	return true;
}


bool GuidanceManager::setArrivalGuide()
{
	Guidance guide;

	//update moving status
	guide.moving_status = m_mvstatus;

	//set actions	
	//if last turn exists,
	if (!isForward(m_finalTurn))
	{
		ExtendedPathElement lastguide = m_extendedPath.back();
		Node* dest = getMap()->getNode(lastguide.cur_node_id);
		if (dest == nullptr)
		{
			printf("[Error] GuidanceManager::setArrivalGuide - undefined last node: %zu!\n", lastguide.cur_node_id);
			return false;
		}
		Motion cmd = getMotion(dest->type, Edge::EDGE_SIDEWALK, m_finalTurn);
		MotionMode mode = getMode(Edge::EDGE_SIDEWALK);
		Action action(cmd, dest->type, Edge::EDGE_SIDEWALK, m_finalTurn, mode);
		guide.actions.push_back(action);
		guide.msg = getStringTurn(action, dest->type) + " and ";
	}

	guide.guide_status = GuideStatus::GUIDE_ARRIVED;
	guide.actions.push_back(Action(Motion::STOP, Node::NODE_BASIC, Edge::EDGE_SIDEWALK, 0, MotionMode::MOVE_NORMAL));
	guide.heading_node_id = 0;
	guide.distance_to_remain = 0;
	guide.msg = guide.msg + "[GUIDANCE] Arrived!";
	m_curguidance = guide;

	return true;
}


//bool GuidanceManager::setArrivalGuide()
//{
//	ExtendedPathElement lastguide = m_extendedPath.back();
//	Node* dest = getMap()->getNode(lastguide.cur_node_id);
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
//		MotionMode mode = getMode(Edge::EDGE_SIDEWALK);
//		Action action(cmd, Edge::EDGE_SIDEWALK, m_finalTurn, mode);
//		guide.actions.push_back(action);
//		guide.msg = getStringTurn(action, dest->type) + " and ";
//	}
//
//	guide.guide_status = GuideStatus::GUIDE_ARRIVED;
//	guide.actions.push_back(Action(Motion::STOP, Edge::EDGE_SIDEWALK, 0, MotionMode::MOVE_NORMAL));
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
	//guide.actions.push_back(Action(Motion::STOP, Edge::EDGE_SIDEWALK, 0, MotionMode::MOVE_NORMAL));
	guide.distance_to_remain = 0;
	guide.msg = "";

	m_curguidance = guide;

	return true;
}


std::string GuidanceManager::getStringFwdDist(Action act, int ntype, ID nid, double d)
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

std::string GuidanceManager::getStringFwdDistAfter(Action act, int ntype, ID nid, double d)
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

	std::string act_add = " after " + distance + "m" + " on " + nodetype + "(Node ID : " + nodeid + ")";
	result = str_act + act_add;
	return result;
}

std::string GuidanceManager::getStringFwd(Action act, int ntype, ID nid)
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
	result = motion + " on " + edge;

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
		result = motion;
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

std::string GuidanceManager::getStringGuidance(Guidance guidance, MovingStatus status)
{
	std::string result, str_first;
	std::vector<std::string> str;
	std::vector<Action> actions = guidance.actions;

	int cnt = 0;
	for (size_t i = 0; i < actions.size(); i++)
	{
		if (isForward(actions[i].cmd))
		{
			if (i == 0)
			{
				
				if (guidance.moving_status == MovingStatus::APPROACHING_NODE)
				{
					str_first = getStringFwdDistAfter(actions[i], actions[i].node_type,
					guidance.heading_node_id, guidance.distance_to_remain);
				}
				else
				{
					str_first = getStringFwdDist(actions[i], actions[i].node_type,
					guidance.heading_node_id, guidance.distance_to_remain);
				}
				
					
			}
			else
			{
				str_first = getStringFwd(actions[i], actions[i].node_type,
					guidance.heading_node_id);
			}
			
			
		}
		else if (actions[i].cmd == Motion::STOP)
		{
			str_first = "Stop!";
		}
		else 
		{
			if (m_mvstatus == MovingStatus::ON_NODE)
				str_first = getStringTurn(actions[i], actions[i].node_type);
			else
				str_first = getStringTurnDist(actions[i], actions[i].node_type, guidance.distance_to_remain);
		}
		str.push_back(str_first);
	}

	//result = "[Guide] " + str[0];

	result = "[Guide] [" + m_movestates[(int)status] + "]";
	if (str.size() > 0) result += str[0];
	if (actions.size() >= 2)	//only 2 steps are shown in msg
		result = result + " and " + str[1];

	return result;
}

bool GuidanceManager::isNodeInPath(ID nodeid)
{
	for (size_t i = 0; i < m_extendedPath.size(); i++)
	{
		if (nodeid == m_extendedPath[i].cur_node_id)
			return true;
	}
	return false;
}

bool GuidanceManager::isEdgeInPath(ID edgeid)
{
	for (size_t i = 0; i < m_extendedPath.size(); i++)
	{
		if (edgeid == m_extendedPath[i].cur_edge_id)
			return true;
}
 	return false;
}

int GuidanceManager::getGuideIdxFromPose(TopometricPose pose)
{
	ID nodeid = pose.node_id;
	for (size_t i = 0; i < m_extendedPath.size(); i++)
	{
		if (nodeid == m_extendedPath[i].cur_node_id)
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


/**
* Get relative angle of p2p3 w.r.t. the direction of p1p2 (Unit: [deg])
*/
int GuidanceManager::getDegree(Point2 p1, Point2 p2, Point2 p3)
{
	Point2 v1 = p2 - p1;
	Point2 v2 = p3 - p2;
	if (norm(v1) <= 0 || norm(v2) <= 0) return 0;

	double ang_rad = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));
	if (v1.cross(v2) < 0) ang_rad = -ang_rad;
	double ang_deg = cx::cvtRad2Deg(ang_rad);

	return (int)ang_deg;
}

bool GuidanceManager::validatePath(const Path& path, const Map& map)
{
	for (size_t i = 0; i < path.pts.size(); i++)
	{
		ID nid = path.pts[i].node_id;
		if (nid != 0 && map.getNode(nid) == nullptr)
		{
			printf("No Node-%zu found on the map!\n", nid);
			return false;
		}

		ID eid = path.pts[i].edge_id;
		if (eid != 0 && map.getEdge(eid) == nullptr)
		{
			printf("No Edge-%zu found on the map!\n", eid);
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
	Node* dest = getMap()->getNode(m_path.pts.back().id);
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
	getMap()->set_union(map);
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