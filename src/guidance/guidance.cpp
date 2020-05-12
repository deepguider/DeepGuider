#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

bool dg::GuidanceManager::validatePath(dg::Path path, dg::Map map)
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
	Node *curNode, *nextNode, *afterNextNode;
	ID curnid, cureid, nextnid, nexteid, aftnextnid;
	while (iter != last)
	{
		//current node
		tempElement = *iter;
		curnid = tempElement.node_id;
		curNode = m_map.findNode(curnid);
		cureid = tempElement.edge_id;

		//next node
		iter++;
		if (iter == last) goto ADD_LAST_NODE;
		tempElement = *iter;
		nextnid = tempElement.node_id;
		nextNode = m_map.findNode(nextnid);
		nexteid = tempElement.edge_id;

		GuidedPathElement tmppath(curnid, cureid, nextnid, nexteid, angle);
		m_guides.push_back(tmppath);

		//calculate degree for next round		
		iter++;	//after next node
		if (iter == last) goto ADD_LAST_NODE;
		tempElement = *iter;
		aftnextnid = tempElement.node_id;
		afterNextNode = m_map.findNode(aftnextnid);
		angle = getDegree(curNode, nextNode, afterNextNode);
		iter--;

	}//while (curnode != dest_node)

ADD_LAST_NODE:
	//add last node
	m_guides.push_back(GuidedPathElement(
		m_path.pts.back().node_id, 0, 0, 0, 0));

	m_guide_idx = -1;
	getNormalGuidance(MoveStatus::ON_NODE);

	return true;

}

bool GuidanceManager::regeneratePath(double start_lat, double start_lon, double dest_lat, double dest_lon)
{
	//replace to new path
	MapManager map_manager;
	dg::Path path;
	map_manager.getPath(start_lat, start_lon, dest_lat, dest_lon, path);
	
	dg::Map map = map_manager.getMap();
	m_map.set_union(map);
	setPathNMap(path, m_map);

	//restart with index 0
	initializeGuides();
	return true;
}

bool GuidanceManager::isNodeInPath(ID nodeid)
{
	for (size_t i = 0; i < m_path.pts.size(); i++)
	{
		if (nodeid == m_path.pts[i].node_id)
		{
			return true;
		}
	}
	return false;
}

GuidanceManager::GuideStatus GuidanceManager::getGuidanceStatus(TopometricPose pose, double conf)
{
	//Current robot location
	ID nodeid = pose.node_id;

	if (isNodeInPath(nodeid))	
	{//as long as nodeid exists on path, everything is ok

		//finishing condition
		if (nodeid == m_path.pts.back().node_id)
		{
			return GuideStatus::GUIDE_ARRIVED;
		}
		oop_start = 0;
		return GuideStatus::GUIDE_NORMAL;
	}
	else
	{
		if (conf >= 0.5) //out-of-path 
		{//robot is in another path, for sure.
			fprintf(stdout, "The node(%zu) is out-of-path!\n", nodeid);
			if (oop_start == 0)	//start timer
				oop_start = time(NULL);

			oop_end = time(NULL);	
			double diff_t = difftime(oop_end, oop_start);
			//timer is already running
			if (diff_t > 5.0)	//after 5 seconds
				return GuideStatus::GUIDE_OOP;
			return GuideStatus::GUIDE_OOP_DETECT;
		}
		else 
		{//Lost, out-of-map (confidence is low or conf == -1)
			//localizer does not know where the robot is.
			//It needs to go back by recovery mode
			return GuideStatus::GUIDE_LOST;
		}
	}
	
}

GuidanceManager::Guidance GuidanceManager::getGuidance(TopometricPose pose)
{
	MoveStatus curStatus = applyPose(pose);
	int gidx = getGuideIdxFromPose(pose);
	if (gidx != m_guide_idx)
	{
		//m_guide_idx = gidx;
		if (0.1 < pose.dist && pose.dist < 1.0)
		{
			curStatus = MoveStatus::ON_EDGE;	//to update guide
		}
		else
		{
			curStatus = MoveStatus::ON_NODE;	//to update guide
		}
	}

	Guidance guide;
	if (!isNodeInPath(pose.node_id))
	{
		guide.msg = std::to_string(pose.node_id) + " is out-of-path";
		return guide;
	}

	guide = getNormalGuidance(curStatus);
	return guide;
}


GuidanceManager::Guidance GuidanceManager::getGuidance(TopometricPose pose, GuideStatus gStatus)
{
	Guidance result;
	MoveStatus ms;

	switch (gStatus)
	{
	case GuideStatus::GUIDE_NORMAL:
		ms = applyPose(pose);
		result = getNormalGuidance(ms);
		break;
	case GuideStatus::GUIDE_ARRIVED:
		result = getNormalGuidance(MoveStatus::ARRIVED);
		break;
	case GuideStatus::GUIDE_OOP_DETECT:
		result = getNormalGuidance(MoveStatus::ON_EDGE);
		break;
	case GuideStatus::GUIDE_OOP:
	{	//need new map and generate path from current pose
		//take original goal and change start to current pose
		double start_lat, start_lon, dest_lat, dest_lon;
		LatLon curGPS = getPoseGPS();
		start_lat = curGPS.lat;
		start_lon = curGPS.lon;
		Node* dest = m_map.findNode(m_path.pts.back().node_id);
		dest_lat = dest->lat;
		dest_lon = dest->lon;
		regeneratePath(start_lat, start_lon, dest_lat, dest_lon);
		result = getNormalGuidance(MoveStatus::ON_NODE);
		break;
	}		
	case GuideStatus::GUIDE_LOST:
		break;
	case GuideStatus::GUIDE_RECOVERY:
		break;
	case GuideStatus::GUIDE_EXPLORATION:
		break;
	case GuideStatus::GUIDE_OPTIMAL_VIEW:
		break;
	default:
		break;
	}

	return result;
}

GuidanceManager::MoveStatus GuidanceManager::applyPose(TopometricPose  pose)
{
	//Current robot location
	ID nodeid = pose.node_id;
	Node* curnode = m_map.findNode(nodeid);
	if (curnode == nullptr)
	{
		m_mvstatus = MoveStatus::ON_EDGE;
		return m_mvstatus;
	}

	ID eid = curnode->edge_ids[pose.edge_idx];
	Edge* curedge = m_map.findEdge(eid);
	double edgedist = curedge->length;
	double curdist = pose.dist;
	//double progress = curdist; //will be exchanged
	double progress = curdist / edgedist; 

	/**Check progress.
	m_edge_progress: 0.0 ~ 1.0
	m_edge_progress indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/

	GuidedPathElement curGP = m_guides[m_guide_idx];

	//Check edge following status
	if ((nodeid != curGP.from_node_id) && (progress < 0.01 ))
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

	return m_mvstatus;
}

GuidanceManager::Motion GuidanceManager::getMotion(int ntype, int etype, int degree)
{
	GuidanceManager::Motion motion;
	std::string rotation;

	if (degree >= -45 && degree <= 45)	//"FORWARD"
	{		
		if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_FORWARD;
		else if (ntype == Node::NODE_DOOR)	// && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_FORWARD;
		//else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			//motion = GuidanceManager::Motion::EXIT_FORWARD;
		else
			motion = GuidanceManager::Motion::GO_FORWARD;
	}
	else if (degree > 45 && degree <= 135)	//"LEFT"
	{
		if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_LEFT;
		else if (ntype == Node::NODE_DOOR)	// && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_LEFT;
		//else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			//motion = GuidanceManager::Motion::EXIT_LEFT;
		else
			motion = GuidanceManager::Motion::TURN_LEFT;
	}
	else if (degree < -45 && degree >= -135)	//"RIGHT"
	{
		if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_RIGHT;
		else if (ntype == Node::NODE_DOOR)	// && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_RIGHT;
		//else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			//motion = GuidanceManager::Motion::EXIT_RIGHT;
		else
			motion = GuidanceManager::Motion::TURN_RIGHT;
	}
	else //"BACK"
		motion = GuidanceManager::Motion::TURN_BACK;

	return motion;	
}

GuidanceManager::Action GuidanceManager::setAction(Motion cmd, int etype, int degree, Mode mode)
{
	Action result;
	result.cmd = cmd;
	result.edge_type = etype;
	result.degree = degree;
	result.mode = mode;
	return result;
}


GuidanceManager::Action GuidanceManager::setAction(ID nid, ID eid, int degree)
{
	Action result;
	Node* node = m_map.findNode(nid);
	Edge* edge = m_map.findEdge(eid);
	Motion cmd = getMotion(node->type, edge->type, degree);
	Mode mode = getMode(edge->type);
	result = setAction(cmd, edge->type, degree, mode);
	return result;
}

GuidanceManager::Guidance GuidanceManager::getNormalGuidance(MoveStatus status)
{
	Guidance guide;
	GuidedPathElement curGP, nextGP;

	//update dgstatus
	guide.guide_status = m_dgstatus;

	//update moving status
	guide.moving_status = status;

	//set actions
	switch (status)
	{
	case MoveStatus::ARRIVED:
	{
		Guidance pastG = getLastGuidance();
		Action action = pastG.actions.back();
		//if last turn exists,
		if (!isForward(pastG.actions.back().cmd))
		{
			guide.actions.push_back(action);
			guide.heading_node_id = m_guides.back().from_node_id;
			guide.msg = getStringGuidance(guide, status);
		}
		guide.guide_status = GuideStatus::GUIDE_ARRIVED;
		guide.actions.push_back(setAction(Motion::STOP,
			Edge::EDGE_SIDEWALK, pastG.actions.back().degree, Mode::MOVE_NORMAL));
		guide.distance_to_remain = 0;
		guide.msg = guide.msg + " Arrived!";
		return guide;
	}
	case MoveStatus::ON_NODE: 		//add next action
	{
		//if past degree is turn, add former turn on current node
		if (m_guide_idx != -1) //check first guide
		{
			GuidedPathElement pastGP = getCurGuidedPath(m_guide_idx);	//from last guide
			if (!isForward(pastGP.degree))	//if TURN exists in past node
			{
				Edge* nextEdge = m_map.findEdge(pastGP.next_edge_id);
				if (nextEdge == nullptr)
				{
					Node* node = m_map.findNode(pastGP.to_node_id);
					Motion cmd = getMotion(node->type, Edge::EDGE_SIDEWALK, pastGP.degree);
					guide.actions.push_back(setAction(
						cmd, Edge::EDGE_SIDEWALK, pastGP.degree, Mode::MOVE_NORMAL));
				}
				else
				{
					guide.actions.push_back(setAction(pastGP.to_node_id, pastGP.next_edge_id, pastGP.degree));
				}
			}
		}

		//add GO on current node
		int gidx = m_guide_idx + 1;
		curGP = getCurGuidedPath(gidx);
		guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));

		//add final TURN on next node
		if (!isForward(curGP.degree))
		{
			Edge* nextEdge = m_map.findEdge(curGP.next_edge_id);
			if (nextEdge == nullptr)
			{
				Node* node = m_map.findNode(curGP.from_node_id);
				Motion cmd = getMotion(node->type, Edge::EDGE_SIDEWALK, curGP.degree);
				guide.actions.push_back(setAction(
					cmd, Edge::EDGE_SIDEWALK, curGP.degree, Mode::MOVE_NORMAL));
			}
			else
			{
				guide.actions.push_back(setAction(curGP.to_node_id, curGP.next_edge_id, curGP.degree));
			}
		}

		//save guidances
		m_past_guides.push_back(guide);
		m_guide_idx++;
		break;
	}
	case MoveStatus::ON_EDGE://maintain current guide
	{	//only forward action is extracted on edge
		curGP = getCurGuidedPath(m_guide_idx);
		guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));
		break;
	}
	case MoveStatus::APPROACHING_NODE://add next action
	{
		//GO_FORWARD
		curGP = getCurGuidedPath(m_guide_idx);
		guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));

		//if TURN exists on next node,
		if (!isForward(curGP.degree))
			guide.actions.push_back(setAction(curGP.to_node_id, curGP.next_edge_id, curGP.degree));
		break;
	}
	default:
		break;
	}

	//update heading_node
	Node* nextnode = m_map.findNode(curGP.to_node_id);
	guide.heading_node_id = nextnode->id;

	//update distance_to_remain
	Edge* curedge = m_map.findEdge(curGP.cur_edge_id);
	guide.distance_to_remain = curedge->length * (1.0 - m_edge_progress);

	//make guidance string
	guide.msg = getStringGuidance(guide, status);

	return guide;

}

std::string dg::GuidanceManager::getStringForward(Action act, int ntype, ID nid, double d)
{
	std::string result;

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string str_act = motion + " on " + edge;

	std::string nodetype = m_nodes[ntype];
	
	std::string nodeid = std::to_string(nid);
	std::string distance = std::to_string(d);

	std::string act_add = " about " + distance + "m" + " until next " + nodetype + "(Node ID : " + nodeid + ")";
	result = str_act + act_add;
	return result;
}

std::string dg::GuidanceManager::getStringTurn(Action act, int ntype)
{
	std::string result;

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);
	std::string str_act = motion + " for " + degree + " degree";

	std::string nodetype = m_nodes[ntype];

	std::string act_add = " on " + nodetype;
	result = str_act + act_add;
	return result;
}

std::string GuidanceManager::getStringGuidance(Guidance guidance, MoveStatus status)
{
	std::string result, str_first;
	std::vector<std::string> str;
	std::vector<Action> actions = guidance.actions;

	Node* node = m_map.findNode(guidance.heading_node_id);

	int cnt = 0;
	for (size_t i = 0; i < actions.size(); i++)
	{
		if (isForward(actions[i].cmd))
		{
			str_first = getStringForward(actions[i], node->type,
				guidance.heading_node_id, guidance.distance_to_remain);
		}
		else
		{
			str_first = getStringTurn(actions[i], node->type);
		}
		str.push_back(str_first);
	}

	result = "[Guide] " + str[0];
	if (actions.size() >= 2)	//only 2 steps are shown in msg
		result = result + " and " + str[1];

	return result;
}

int dg::GuidanceManager::getGuideIdxFromPose(TopometricPose pose)
{
	ID curnodei = pose.node_id;
	for (size_t i = 0; i < m_guides.size(); i++)
	{
		if (m_guides[i].from_node_id == curnodei)
		{
			return (int)i;
		}
	}
	return -1;
}

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

	double rad2deg = rad / 3.14 * 180.0;
	if (sign < 0.f) rad2deg = -rad2deg;	//this is for -180~180

	int result = (int) rad2deg;

	return result;

}

//Node* dg::GuidanceManager::findNodeFromPath(ID nodeid)
//{
//	for (size_t i = 0; i < m_path.pts.size(); i++)
//	{
//		if (m_path.pts[i].node->id == nodeid)
//		{
//			return m_path.pts[i].node;
//		}
//	}
//	return nullptr;
//}
//
//Edge* dg::GuidanceManager::findEdgeFromPath(ID edgeid)
//{
//	for (size_t i = 0; i < m_path.pts.size(); i++)
//	{
//		if (m_path.pts[i].edge->id == edgeid)
//		{
//			return m_path.pts[i].edge;
//		}
//	}
//	return nullptr;
//}

