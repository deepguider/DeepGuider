#include "guidance.hpp"
#include "dg_map_manager.hpp"
#include "utils/opencx.hpp"

using namespace dg;

bool GuidanceManager::initialize(SharedInterface* shared)
{
	m_shared = shared;

	//initialize guidance parameters
	std::vector <ExtendedPathElement> m_extendedPath;
	GuideStatus  m_gstatus = GuideStatus::GUIDE_NORMAL;
	std::vector<Guidance> m_past_guides;
	Guidance m_curguidance;
	RobotStatus m_robot_status = RobotStatus::READY;
	m_guide_idx = -1;	//starts with -1 because its pointing current guide.
	m_robot_guide_idx = -1;
	m_remain_distance = 0.0;
	m_last_announce_dist = -1;
	m_arrival = false;
	m_arrival_cnt = 0;

	return (m_shared != nullptr);
}

bool GuidanceManager::initiateNewGuidance()
{
	initialize(m_shared);
	return buildGuides();
}

bool GuidanceManager::initiateNewGuidance(TopometricPose pose_topo, Point2F gps_dest)
{
	initialize(m_shared);
	if (buildGuides())
	{
		//add start path		
		ID cur_node_id = pose_topo.node_id;
		Node* curNode = getMap()->getNode(cur_node_id);
		if (curNode == nullptr)
		{
			printf("[Error] GuidanceManager::initiateNewGuidance - Node-%zd is not in the map!\n", cur_node_id);
			return false;
		}
		ID cur_edge_id = curNode->edge_ids[pose_topo.edge_idx];
		ExtendedPathElement start_element(cur_node_id, cur_edge_id, m_extendedPath[0].cur_node_id, m_extendedPath[0].cur_edge_id,
			0, curNode->x, curNode->y);

		double start_dist = norm(m_extendedPath[0] - Point2(curNode->x, curNode->y));
		if (m_extendedPath[0].is_junction)
		{
			start_element.remain_distance_to_next_junction = start_dist;
			start_element.next_guide_node_id = m_extendedPath[0].cur_node_id;
			start_element.next_guide_edge_id = m_extendedPath[0].cur_edge_id;
		}
		else
		{
			start_element.remain_distance_to_next_junction = start_dist + m_extendedPath[0].remain_distance_to_next_junction;
			start_element.next_guide_node_id = m_extendedPath[0].next_guide_node_id;
			start_element.next_guide_edge_id = m_extendedPath[0].next_guide_edge_id;
		}
		m_extendedPath.insert(m_extendedPath.begin(), start_element);

		//add dest path		
		size_t sz = m_extendedPath.size();
		double dest_dist = norm(m_extendedPath.back() - gps_dest);
		for (size_t i = sz - 1; i >= 0; i--)
		{
			m_extendedPath[i].remain_distance_to_next_junction = dest_dist + m_extendedPath[i].remain_distance_to_next_junction;
			if (m_extendedPath[i].is_junction)
				break;
		}

		//add next junction id
		int junction_id = 0;
		for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
		{
			m_extendedPath[i].next_junction_idx = junction_id;
			if (m_extendedPath[i].is_junction)
				junction_id = i;
		}

		ExtendedPathElement dest_element(0, 0, 0, 0, 0, gps_dest.x, gps_dest.y);
		m_extendedPath.push_back(dest_element);

		//print path
		for (int k = 0; k < m_extendedPath.size(); k++)
		{
            Point2 node_metric = Point2(m_extendedPath[k]);
			printf("Extendedpath[%d] node_metric: <%f, %f> \n",k, node_metric.x, node_metric.y);
			printf("Extendedpath[%d] cur_node_id:%zd, next_node_id: %zd\n",k,m_extendedPath[k].cur_node_id, m_extendedPath[k].next_node_id);
		}
		
		return true;
	}
	else
		return false;
}


bool GuidanceManager::initiateNewGuidance(Point2F gps_start, Point2F gps_dest)
{
	if (buildGuides())
	{
		//add start path		
		ExtendedPathElement start_element(0, 0, m_extendedPath[0].cur_node_id, m_extendedPath[0].cur_edge_id,
			0, gps_start.x, gps_start.y);

		double start_dist = norm(m_extendedPath[0] - gps_start);
		if (m_extendedPath[0].is_junction)
		{
			start_element.remain_distance_to_next_junction = start_dist;
			start_element.next_guide_node_id = m_extendedPath[0].cur_node_id;
			start_element.next_guide_edge_id = m_extendedPath[0].cur_edge_id;
		}
		else
		{
			start_element.remain_distance_to_next_junction = start_dist + m_extendedPath[0].remain_distance_to_next_junction;
			start_element.next_guide_node_id = m_extendedPath[0].next_guide_node_id;
			start_element.next_guide_edge_id = m_extendedPath[0].next_guide_edge_id;
		}
		m_extendedPath.insert(m_extendedPath.begin(), start_element);

		//add dest path		
		size_t sz = m_extendedPath.size();
		double dest_dist = norm(m_extendedPath.back() - gps_dest);
		for (size_t i = sz - 1; i >= 0; i--)
		{
			m_extendedPath[i].remain_distance_to_next_junction = dest_dist + m_extendedPath[i].remain_distance_to_next_junction;
			if (m_extendedPath[i].is_junction)
				break;
		}
		ExtendedPathElement dest_element(0, 0, 0, 0, 0, gps_dest.x, gps_dest.y);
		m_extendedPath.push_back(dest_element);

		int junction_id = 0;
		for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
		{
			m_extendedPath[i].next_junction_idx = junction_id;
			if (m_extendedPath[i].is_junction)
				junction_id = i;
		}

		return true;
	}
	else
		return false;
}

bool GuidanceManager::buildGuides()
{
	// check map and path
	Map* map = getMap();
	if (map == nullptr || map->isEmpty())
	{
		printf("[Error] GuidanceManager::buildGuides - Empty Map\n");
		return false;
	}
	Path path = (m_shared) ? m_shared->getPath() : Path();
	if (path.empty())
	{
		printf("[Error] GuidanceManager::buildGuides - Empty path\n");
		return false;
	}
	if (!validatePath(path, *map))
	{
		printf("[Error] GuidanceManager::buildGuides - Path is not in the map!\n");
		return false;
	}

	// clear old guidance path
	m_extendedPath.clear();

	// build new guidance path (exclude first and last path element: temporal start and destination node)
	for (int i = 1; i < (int)path.pts.size() - 1; i++)
	{
		ID curnid = path.pts[i].node_id;
		ID cureid = path.pts[i].edge_id;
		ID nextnid = path.pts[i + 1].node_id;
		ID nexteid = path.pts[i + 1].edge_id;
		Node* curNode = getMap()->getNode(curnid);
		if (curNode == nullptr)
		{
			printf("[Error] GuidanceManager::buildGuides - Node-%zd is not in the map!\n", curnid);
			return false;
		}

		int angle = 0;
		if (i > 0)
		{
			angle = getDegree(path.pts[i - 1], path.pts[i], path.pts[i + 1]);
			//printf("%d\n", angle);
		}
		ExtendedPathElement tmppath(curnid, cureid, nextnid, nexteid, angle, path.pts[i].x, path.pts[i].y);

		if (curNode && (curNode->type == Node::NODE_JUNCTION || curNode->type == Node::NODE_DOOR))
		{
			tmppath.is_junction = true;
		}

		m_extendedPath.push_back(tmppath);
	}

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
	int deg_fromback = 0;
	for (int i = (int)m_extendedPath.size() - 2; i >= 0; i--)
	{
		m_extendedPath[i].next_guide_node_id = next_guide_node_id;
		m_extendedPath[i].next_guide_edge_id = next_guide_edge_id;
		m_extendedPath[i].junction_degree = deg_fromback;

		if (next_guide_node_id == 0 || m_extendedPath[i].is_junction)
		{
			next_guide_node_id = m_extendedPath[i].cur_node_id;
			next_guide_edge_id = m_extendedPath[i].cur_edge_id;
			deg_fromback = m_extendedPath[i].cur_degree;
		}
	}

	m_guide_idx = 0;
	m_robot_guide_idx = 0;
	
	return true;
}


bool GuidanceManager::update(TopometricPose pose, Pose2 pose_metric)
{
	//check pose validation
	ID curnid = pose.node_id;	//current robot's pose
	Node* curNode = getMap()->getNode(curnid);
	if (curNode == nullptr)
	{
		printf("[Error] GuidanceManager::update - curNode: %zu == nullptr!\n", curnid);
		return false;
	}
	ID cureid = curNode->edge_ids[pose.edge_idx];
	Edge* curEdge = getMap()->getEdge(cureid);
	if (curEdge == nullptr)
	{
		printf("[Error] GuidanceManager::update - curEdge: %zu == nullptr!\n", cureid);
		return false;
	}

	//after arrival, continuously moving.
	if (m_arrival)
	{
		if (m_arrival_cnt > m_max_arrival_cnt)
		{
			m_extendedPath.clear();
			m_arrival = false;
			m_arrival_cnt = 0;
			m_gstatus = GuideStatus::GUIDE_NOPATH;
			setEmptyGuide();
			return true;
		}
		m_arrival_cnt++;
	}

	//finally arrived
	double goal_dist = norm(m_extendedPath.back() - pose_metric);
	if (m_guide_idx >= m_extendedPath.size() - 2 && goal_dist < m_arrived_threshold)
	{
		m_gstatus = GuideStatus::GUIDE_ARRIVED;
		m_arrival = true;
		setArrivalGuide();
		m_curguidance.announce = true;
		m_curguidance.distance_to_remain = goal_dist;
		m_arrival_cnt++;
		m_guide_idx = -1;
		m_last_announce_dist = -1;	//reset m_last_announce_dist
		return true;
	}

	//if robot is not on-path
	int gidx = getGuideIdxFromPose(pose);
	if (gidx == -1)
	{
		printf("[Error] GuidanceManager::update - Pose not found on path! %zd\n", pose.node_id);
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		setEmptyGuide();
		return false;
	}

	//if node of pose is changed, update m_guide_idx
	if (gidx != m_guide_idx)
	{
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_last_announce_dist = -1;	//reset m_last_announce_dist
		m_guide_idx = gidx;
	}

	//check remain distance
	ExtendedPathElement curEP = getCurExtendedPath();
	double passsed_dist = pose.dist;
	double junction_dist = curEP.remain_distance_to_next_junction;
	double remain_dist = junction_dist - passsed_dist; // + curedge->length;
	m_remain_distance = remain_dist;

	//check finishing condition
	// if (m_guide_idx >= m_extendedPath.size() - 2 && goal_dist < m_start_exploration_dist)
	// {
	// 	//Optimal view sequence
	// 	m_gstatus = GuideStatus::GUIDE_OPTIMAL_VIEW;	
	// 	setEmptyGuide();
	// 	m_curguidance.msg = "[GUIDANCE] NEAR_ARRIVAL. Optimal view started!";
	// 	m_curguidance.announce = true;
	// 	m_curguidance.distance_to_remain = m_remain_distance;
	// 	return true;
	// }

	//check announce
	int announce_dist = ((int) (remain_dist - 1 ) / m_guide_interval) * m_guide_interval;
	bool announce = false;

	m_gstatus = GuideStatus::GUIDE_NORMAL;
	//on junction
	// if (remain_dist < m_arrived_threshold)
	// {
	// 	m_last_announce_dist = -1;	//reset m_last_announce_dist
	// 	announce = false;
	// 	setEmptyGuide();
	// }
	//near junction
	if (remain_dist <= m_uncertain_dist)// || (curNode->type == Node::NODE_JUNCTION && passsed_dist < m_uncertain_dist))
	{
		//if the edge is shorter than 2*m_uncertain_dist
		if (junction_dist <= 2 * m_uncertain_dist)
		{
			//	printf("m_last_announce_dist: %d, %d, %d\n", m_last_announce_dist, announce_dist, announce);
			if ((m_last_announce_dist != announce_dist) && !announce)
			{
				announce = true;
				m_last_announce_dist = announce_dist;
				setSimpleGuide();
			}
			else
			{
				announce = false;
				setEmptyGuide();
			}
		}
		else
		{
			announce = false;
			setSimpleGuide();
		}
	}
	//normal case
	else
	{
		if (announce_dist != m_last_announce_dist)
		{
			announce = true;
			setSimpleGuide();
			m_last_announce_dist = announce_dist;
		}
		else
			announce = false;
	}
	m_curguidance.announce = announce;
	m_curguidance.distance_to_remain = m_remain_distance;

	return true;
}

bool GuidanceManager::updateWithRobot(TopometricPose pose, Pose2 pose_metric)
{
	//check pose validation
	ID curnid = pose.node_id;	//current robot's pose
	Node* curNode = getMap()->getNode(curnid);
	if (curNode == nullptr)
	{
		printf("[Error] GuidanceManager::update - curNode: %zu == nullptr!\n", curnid);
		return false;
	}
	ID cureid = curNode->edge_ids[pose.edge_idx];
	Edge* curEdge = getMap()->getEdge(cureid);
	if (curEdge == nullptr)
	{
		printf("[Error] GuidanceManager::update - curEdge: %zu == nullptr!\n", cureid);
		return false;
	}

	//after arrival, continuously moving.
	if (m_arrival)
	{
		if (m_arrival_cnt > m_max_arrival_cnt)
		{
			m_extendedPath.clear();
			m_arrival = false;
			m_arrival_cnt = 0;
			m_gstatus = GuideStatus::GUIDE_NOPATH;
			setEmptyGuide();
			printf("[GUIDANCE] Arrived to the goal\n");
			return true;
		}
		m_arrival_cnt++;
	}

	//finally arrived
	double goal_dist = norm(m_extendedPath.back() - pose_metric);
	if(m_robot_status == RobotStatus::ARRIVED_NODE || 
		(m_guide_idx >= m_extendedPath.size() && goal_dist < m_arrived_threshold))
	{
		m_gstatus = GuideStatus::GUIDE_ARRIVED;
		m_arrival = true;
		setArrivalGuide();
		m_curguidance.announce = true;
		m_curguidance.distance_to_remain = goal_dist;
		m_arrival_cnt++;
		m_guide_idx = -1;
		m_robot_guide_idx = -1;
		m_last_announce_dist = -1;	//reset m_last_announce_dist
		return true;
	}

	//if robot is not on-path
	int gidx = getGuideIdxFromPose(pose);
	if (gidx == -1)
	{
		printf("[Error] GuidanceManager::update - Pose not found on path!\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		setEmptyGuide();
		return false;
	}

    Pose2 node_metric = Pose2(curNode->x, curNode->y);
	//from here gidx > 0
	//if robot has arrived at the node, update m_guide_idx 	
	//printf("[GUIDANCE] gidx: %d, m_guide_idx: %d\n", gidx, m_guide_idx);
	//printf("[GUIDANCE] updateWithRobot: %d, %zd\n", isNodeInPastGuides(m_curguidance.heading_node_id), m_curguidance.heading_node_id);
	// printf("[GUIDANCE] m_robot_guide_idx: %d, isRobotNearArrivalNode: %d\n", m_robot_guide_idx, isRobotNearArrivalNode());
	if (m_robot_status == RobotStatus::ARRIVED_NODE && isRobotNearArrivalNode())
	// if (m_robot_status == RobotStatus::ARRIVED_NODE)
	//if((m_robot_status == RobotStatus::ARRIVED_NODE) && !isNodeInPastGuides(m_curguidance.heading_node_id))
	{
		printf("[GUIDANCE] updateWithRobot::ARRIVED_NODE, gidx: %d, m_guide_idx: %d, m_robot_guide_idx:%d\n", gidx, m_guide_idx, m_robot_guide_idx);
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_prev_dg_pose = node_metric;
		m_last_announce_dist = -1;	//reset m_last_announce_dist
		m_guide_idx = m_robot_guide_idx + 1 ;
		m_temp2_prev_dg_pose = pose_metric;
		m_prev_dx_pose = m_temp_prev_dx_pose;
		m_temp2_prev_dx_pose = m_robot_pose;
		printf("[GUIDANCE] updateWithRobot::ARRIVED_NODE, gidx: %d, m_guide_idx: %d, m_robot_guide_idx:%d\n", gidx, m_guide_idx, m_robot_guide_idx);
		printf("[GUIDANCE] updateWithRobot::ARRIVED_NODE, heading_node: %zd\n", m_extendedPath[m_guide_idx].cur_node_id);
	}
	
	if (gidx > m_guide_idx) //dg_pose is ahead of robot
	{
		printf("[GUIDANCE] gidx != m_guide_idx, gidx: %d, m_guide_idx: %d\n", gidx, m_guide_idx);
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_last_announce_dist = -1;	//reset m_last_announce_dist
		m_guide_idx = gidx;
	}

	if (m_robot_status == RobotStatus::RUN_MANUAL || m_robot_status == RobotStatus::RUN_AUTO)
	{
		printf("[GUIDANCE] RobotStatus::RUN_\n");
		m_robot_guide_idx = m_guide_idx;
		m_temp_prev_dg_pose = m_temp2_prev_dg_pose;
		m_temp_prev_dx_pose = m_temp2_prev_dx_pose;
	}
	

	//check remain distance
	ExtendedPathElement curEP = getCurExtendedPath();
	double passsed_dist = pose.dist;
	double junction_dist = curEP.remain_distance_to_next_junction;
	double remain_dist = junction_dist - passsed_dist; // + curedge->length;
	m_remain_distance = remain_dist;

	//final goal
	if (curEP.next_node_id = 0 && curEP.x != 0)
	{
		printf("[GUIDANCE] Approaching Final Goal!\n");
	}
	//check finishing condition
	// if (m_guide_idx >= m_extendedPath.size() - 2 && goal_dist < m_start_exploration_dist)
	// {
	// 	//Optimal view sequence
	// 	m_gstatus = GuideStatus::GUIDE_OPTIMAL_VIEW;	
	// 	setEmptyGuide();
	// 	m_curguidance.msg = "[GUIDANCE] NEAR_ARRIVAL. Optimal view started!";
	// 	m_curguidance.announce = true;
	// 	m_curguidance.distance_to_remain = m_remain_distance;
	// 	return true;
	// }

	m_gstatus = GuideStatus::GUIDE_NORMAL;
	setSimpleGuide();
	m_curguidance.announce = false;
	m_curguidance.distance_to_remain = m_remain_distance;

	return true;
}

bool GuidanceManager::setEmptyGuide()
{
	m_curguidance = Guidance();
	return true;
}

bool GuidanceManager::setSimpleGuide()
{
	Guidance guide;
	guide.guide_status = m_gstatus;

	ExtendedPathElement curEP = getCurExtendedPath();
	ExtendedPathElement nextJuncEP = getExtendedPath(curEP.next_junction_idx);
			
	if (m_remain_distance > m_uncertain_dist )
	{
		
		//first action
		guide.actions.push_back(setActionGo(curEP.next_node_id, curEP.cur_edge_id));
	
		//second action
		if (curEP.next_junction_idx != 0)
		{
			//if junction turn exists
			if (!isForward(nextJuncEP.cur_degree))
			{
				guide.actions.push_back(
					setActionTurn(nextJuncEP.cur_node_id, nextJuncEP.cur_edge_id, nextJuncEP.cur_degree));
			}
			else
				guide.actions.push_back(setActionGo(nextJuncEP.next_node_id, nextJuncEP.cur_edge_id));
		}
		else //if(curEP.next_junction_idx == 0)	//if next juction is final goal
		{
			Action act(Motion::STOP, 0,0,0, MotionMode::MOVE_NORMAL);
			guide.actions.push_back(act);
		}
		
	}	
	else //near junction
	{
		//if junction turn exists
		if (!isForward(nextJuncEP.cur_degree))
		{
			guide.actions.push_back(
				setActionTurn(nextJuncEP.cur_node_id, nextJuncEP.cur_edge_id, nextJuncEP.cur_degree));
		}
		else
		{
			guide.actions.push_back(setActionGo(nextJuncEP.next_node_id, nextJuncEP.cur_edge_id));
		}
		
	}

	guide.heading_node_id = curEP.next_node_id;
	guide.distance_to_remain = m_remain_distance;
	guide.msg = getStringGuidance(guide);
	m_curguidance = guide;
	return true;
}

bool GuidanceManager::setArrivalGuide()
{
	Guidance guide;
	guide.guide_status = GuideStatus::GUIDE_ARRIVED;

	guide.actions.push_back(Action(Motion::STOP, Node::NODE_BASIC, Edge::EDGE_SIDEWALK, 0, MotionMode::MOVE_NORMAL));

	guide.heading_node_id = 0;
	guide.distance_to_remain = 0;
	guide.msg = guide.msg + "Arrived!";
	m_curguidance = guide;

	return true;
}

bool GuidanceManager::isRobotNearArrivalNode()
{
	double remain_dist = norm(m_robot_heading_node_pose - m_robot_pose);
	if (remain_dist < m_uncertain_dist)
	{
		printf("isRobotNearArrivalNode-norm: %f\n", remain_dist);
		return true;
	}
	
	return false;
}

bool GuidanceManager::isDGNearArrivalNode()
{
	double remain_dist = norm(m_robot_heading_node_pose - m_dg_pose);
	if (remain_dist < m_uncertain_dist)
	{
		printf("isRobotNearArrivalNode-norm: %f\n", remain_dist);
		return true;
	}
	
	return false;
}


GuidanceManager::ExtendedPathElement GuidanceManager::getExtendedPath(int idx)
{
	if (idx < 0 || idx >= (int)m_extendedPath.size())
	{
		printf("[ERROR] GuidanceManager::getCurExtendedPath: idx = %d\n", idx);
		return GuidanceManager::ExtendedPathElement();
	}
	return m_extendedPath[idx];
}


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
	Motion cmd;
	MotionMode mode;
	Action result;
	if (node == nullptr || edge == nullptr)
	{
        if (isExpendedPathGenerated())
        {
            Point2 node_metric = Point2(getCurExtendedPath());
            //last guide
            if (nid_next == 0 && node_metric.x != 0)
            {
				cmd = GuidanceManager::Motion::GO_FORWARD;
				result = Action(Motion::GO_FORWARD, Node::NODE_BASIC, Edge::EDGE_SIDEWALK, 0, MotionMode::MOVE_NORMAL);
			}
			else
			{
				printf("[Error] GuidanceManager::setActionGo - No node or edge\n");
				return GuidanceManager::Action();
			}
		}		
	}
	else
	{
		cmd = getMotion(node->type, edge->type, degree);
		mode = getMode(edge->type);
		result = Action(cmd, node->type, edge->type, degree, mode);
	}
	
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
			motion = GuidanceManager::Motion::ENTER_FORWARD;
		else if (degree > 30 && degree <= 150)	//"LEFT"
			motion = GuidanceManager::Motion::ENTER_LEFT;
		else if (degree < -30 && degree >= -150)	//"RIGHT"
			motion = GuidanceManager::Motion::ENTER_RIGHT;
	}
	else
	{
		if (degree > 150 || degree < -150) //"BACK"
			motion = GuidanceManager::Motion::TURN_BACK;
		else
			motion = GuidanceManager::Motion::GO_FORWARD;
	}

	return motion;
}

std::string GuidanceManager::getStringGuidance(Guidance guidance)
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
				str_first = getStringFwdDist(actions[i], actions[i].node_type,
					guidance.heading_node_id, guidance.distance_to_remain);
			}
			else
			{
				str_first = getStringFwd(actions[i], actions[i].node_type,
					guidance.heading_node_id);	
			}
		}
		else if (actions[i].cmd == Motion::STOP)
		{
			str_first = "Arrival";
		}		
		else
		{
			str_first = getStringTurn(actions[i], actions[i].node_type);
			//str_first = getStringTurnDist(actions[i], actions[i].node_type, guidance.distance_to_remain);
		}
		str.push_back(str_first);
	}

	//result = "[Guide]";
	if (str.size() > 0) result += str[0];
	if (actions.size() >= 2)	//only 2 steps are shown in msg
		result = result + " and " + str[1];

	return result;
}

std::string GuidanceManager::getStringFwdDist(Action act, int ntype, ID nid, double d)
{
	std::string result;

	//check parameter
	if ((int)act.cmd < 0 || act.cmd >= Motion::TYPE_NUM || (int)act.edge_type < 0 || act.edge_type >= Edge::TYPE_NUM)
	{
		printf("[Error] GuidanceManager::getStringForward() - wrong Action!\n");
		return result;
	}

	std::string motion = m_motions[(int)act.cmd];
	motion.replace(motion.find("_"), 1, " ");
	std::string edge = m_edges[act.edge_type];
	std::string str_act = motion + " on " + edge;
	std::string nodetype = m_nodes[ntype];
	std::string nodeid = std::to_string(nid);
	std::string distance = std::to_string((int) d);
	//std::string distance = std::to_string(d).substr(0, 4);

	//std::string act_add = " about " + distance + "m" + " until next " + nodetype + "(Node ID : " + nodeid + ")";
	std::string act_add = " for " + distance + "m";
	result = str_act + act_add;

	return result;
}

std::string GuidanceManager::getStringFwd(Action act, int ntype, ID nid)
{
	std::string result;

	//check parameter
	if ((int)act.cmd < 0 || act.cmd >= Motion::TYPE_NUM || (int)act.edge_type < 0 || act.edge_type >= Edge::TYPE_NUM)
	{
		printf("[Error] GuidanceManager::getStringForward() - wrong Action!\n");
		return result;
	}

	std::string motion = m_motions[(int)act.cmd];
	motion.replace(motion.find("_"), 1, " ");
	std::string edge = m_edges[act.edge_type];
	//result = motion + " on " + edge;
	result = motion;

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
	motion.replace(motion.find("_"), 1, " ");
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);
	//std::string str_act = motion + " for " + degree + " degree";
	std::string str_act = motion;
	result = str_act;
	//std::string nodetype = m_nodes[ntype];
	//std::string act_add = " on " + nodetype;
	//result = str_act + act_add;
	return result;
}

std::string GuidanceManager::getStringTurnDist(Action act, int ntype, double dist)
{
	std::string result;

	std::string motion = m_motions[(int)act.cmd];
	motion.replace(motion.find("_"), 1, " ");
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);

	if (act.cmd == Motion::TURN_BACK)
	{
		result = motion;
	}
	else
	{
		std::string distance = std::to_string((int) dist);
		std::string str_act = motion + " after " + distance + "m ";
		std::string nodetype = m_nodes[ntype];
		result = str_act;
		//std::string act_add = " on " + nodetype;
		//result = str_act + act_add;
	}

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

bool GuidanceManager::isNodeInPastGuides(ID nodeid)
{
	for (size_t i = 0; i < m_past_guides.size(); i++)
	{
		if (nodeid == m_past_guides[i].heading_node_id)
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
			return (int)i;
	}
	return -1;
}


// cv::Point2d GuidanceManager::cvtWorld2Image(cv::Point2d val, double deg, cv::Point2d px_per_val, cv::Point2d offset)
// {	
// 	cv::Point2d px;
// 	double cost = cos(-cx::cvtDeg2Rad(deg));
// 	double sint = sin(-cx::cvtDeg2Rad(deg));
// 	px.x = (val.x * px_per_val.x) * cost - (-val.y * px_per_val.y) * sint + offset.x;
// 	px.y = (val.x * px_per_val.x) * sint + (-val.y * px_per_val.y) * cost + offset.y;

// 	return px;
// }

// cv::Point2d GuidanceManager::cvtWorld2ImageRad(cv::Point2d val, double rad, cv::Point2d px_per_val, cv::Point2d offset)
// {	
// 	cv::Point2d px;
// 	double cost = cos(-rad);
// 	double sint = sin(-rad);
// 	px.x = (val.x * px_per_val.x) * cost - (-val.y * px_per_val.y) * sint + offset.x;
// 	px.y = (val.x * px_per_val.x) * sint + (-val.y * px_per_val.y) * cost + offset.y;

// 	return px;
// }

// cv::Point2d GuidanceManager::cvtImage2World(cv::Point2d px, double deg, cv::Point2d px_per_val, cv::Point2d offset)
// {
// 	CV_DbgAssert(px_per_val.x > 0 && px_per_val.y > 0);

// 	cv::Point2d val;
// 	double cost = cos(-cx::cvtDeg2Rad(deg));
// 	double sint = sin(-cx::cvtDeg2Rad(deg));
// 	val.x = ((px.x - px_per_val.x) * cost + (px.y - px_per_val.y) * sint) / px_per_val.x  + offset.x;
// 	val.y = -(-(px.x - px_per_val.x) * sint + (px.y - px_per_val.y) * cost) / px_per_val.y + offset.y;

// 	return val;
// }

// cv::Point2d GuidanceManager::cvtRobot2World(cv::Point2d val, double deg, cv::Point2d px_per_val, cv::Point2d offset)
// {	
// 	cv::Point2d px;
// 	double cost = cos(-cx::cvtDeg2Rad(deg));
// 	double sint = sin(-cx::cvtDeg2Rad(deg));
// 	px.x = (val.x * px_per_val.x) * cost - (val.y * px_per_val.y) * sint + offset.x;
// 	px.y = (val.x * px_per_val.x) * sint + (val.y * px_per_val.y) * cost + offset.y;

// 	return px;
// }

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
