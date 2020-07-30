#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

bool GuidanceManager::validatePath(dg::Path path, dg::Map map)
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
	if (m_map.nodes.empty())
	{
		printf("[Error] GuidanceManager::initializeGuides] Empty Map\n");
		return false;
	}
	if (m_path.pts.size() < 1)
	{
		printf("[Error] GuidanceManager::initializeGuides] Empty path\n");
		return false;
	}

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
	int finalTurn = 90;
	m_guides.push_back(GuidedPathElement(
		m_path.pts.back().node_id, 0, 0, 0, finalTurn));

	m_guide_idx = 0;
	setInitialGuide();

	return true;

}

bool GuidanceManager::update(TopometricPose pose, double conf)
{
	//validate parameters
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::updateGuidance] Empty pose\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		return false;
	}

	if (applyPose(pose))
		if (setGuidanceStatus(pose, conf))
			if (setGuidanceWithGStatus())
				return true;

	setEmptyGuide();
	return false;
}

bool GuidanceManager::setGuidanceStatus(TopometricPose pose, double conf)
{
	//Current robot location
	ID nodeid = pose.node_id;

	//validate parameters
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::updateGuidance] Empty pose\n");
		m_gstatus = GuideStatus::GUIDE_UNKNOWN;
		return false;
	}

	if (m_guide_idx == 0)
	{
		m_gstatus = GuideStatus::GUIDE_INITIAL;
		return true;
	}

	if (isNodeInPath(nodeid))
	{//as long as nodeid exists on path, everything is ok

		//finishing condition
		if (nodeid == m_path.pts.back().node_id)
		{
			m_gstatus = GuideStatus::GUIDE_ARRIVED;
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
				return true;

			}
			printf("The node(%zu) is out-of-path detected!\n", nodeid);
			m_gstatus = GuideStatus::GUIDE_OOP_DETECT;
			return true;
		}
		else
		{//Lost, out-of-map (confidence is low or conf == -1)
			//localizer does not know where the robot is.
			//It needs to go back by recovery mode
			//m_gstatus = GuideStatus::GUIDE_LOST;
			//return true;
		}
	}
	m_gstatus = GuideStatus::GUIDE_NORMAL;
	return false;
}


/**applyPose updates pose related variables.
	(m_mvstatus, m_guide_idx, m_distance)
*/
bool GuidanceManager::applyPose(TopometricPose  pose)
{
	//validate parameter
	if (pose.node_id == 0)
	{
		printf("[Error] GuidanceManager::applyPose] Empty pose!\n");
		m_mvstatus = MoveStatus::STOP_WAIT;
		return false;
	}

	//validate Current robot location
	ID nodeid = pose.node_id;
	Node* curnode = m_map.findNode(nodeid);
	if (curnode == nullptr)
	{
		printf("[Error] GuidanceManager::applyPose] The node is out-of-map!\n");
		m_mvstatus = MoveStatus::ON_EDGE;
		return false;
	}

	//update m_guide_idx
	int gidx = getGuideIdxFromPose(pose);
	if (gidx == -1)
	{
		printf("[Error] GuidanceManager::applyPose] The node is out-of-path!\n");
		m_mvstatus = MoveStatus::ON_EDGE;
	}
	else if (gidx != m_guide_idx)//if new node appears
	{
		m_past_guides.push_back(m_curguidance); //save past guidances
		m_guide_idx = gidx;
	}

	//check remain distance
	ID eid = curnode->edge_ids[pose.edge_idx];
	Edge* curedge = m_map.findEdge(eid);
	double edgedist = curedge->length;
	double curdist = pose.dist;
	m_edge_progress = curdist / edgedist;
	m_rmdistance = edgedist - curdist;

	/**Check progress.
	m_edge_progress: 0.0 ~ 1.0
	m_edge_progress indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/

	//Check edge following status
	if (m_edge_progress >= 0.8)
	{
		m_mvstatus = MoveStatus::APPROACHING_NODE;
		if (m_rmdistance < 1.0)
			m_mvstatus = MoveStatus::ON_NODE;
	}
	else
		m_mvstatus = MoveStatus::ON_EDGE;

	return true;
}

bool GuidanceManager::setGuidanceWithGStatus()
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
		setOOPGuide();
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
		setNormalGuide();
		break;
	}

	return true;
}

bool GuidanceManager::setInitialGuide()
{
	Guidance guide;

	//update GuideStatus
	guide.guide_status = GuideStatus::GUIDE_NORMAL;

	//update moving status
	guide.moving_status = MoveStatus::ON_EDGE;

	GuidedPathElement curGP = getCurGuidedPath(0);
	guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));

	//update heading_node
	Node* nextnode = m_map.findNode(curGP.to_node_id);
	guide.heading_node_id = nextnode->id;

	//update distance_to_remain
	Edge* curedge = m_map.findEdge(curGP.cur_edge_id);
	guide.distance_to_remain = curedge->length ;

	//make guidance string
	guide.msg = getStringGuidance(guide, MoveStatus::ON_EDGE);

	m_curguidance = guide;

	return true;
}

bool GuidanceManager::setNormalGuide()
{
	Guidance guide;
	GuidedPathElement pastGP = getCurGuidedPath(m_guide_idx - 1);
	GuidedPathElement curGP = getCurGuidedPath(m_guide_idx);

	//update dgstatus
	guide.guide_status = m_gstatus;

	//update moving status
	guide.moving_status = m_mvstatus;

	//set actions
	switch (m_mvstatus)
	{
	case MoveStatus::ON_NODE: //(TURN) - GO - (TURN)
	{
		//if TURN exists on past node, add former turn on current node//from last guide
		if (!isForward(pastGP.degree))	//if TURN exists in past node
			guide.actions.push_back(addTurnAction(pastGP));

		//add GO on current node
		guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));

		//if TURN exists on next node, add final TURN on next node
		if (!isForward(curGP.degree))
			guide.actions.push_back(addTurnAction(curGP));

		break;
	}
	case MoveStatus::ON_EDGE://maintain current guide
	{	//only forward action is displayed on edge status
		guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));
		break;
	}
	case MoveStatus::APPROACHING_NODE://After 000m, (TURN) - GO
	{
		//if TURN exists on next node,
		if (!isForward(curGP.degree))
			guide.actions.push_back(setAction(curGP.to_node_id, curGP.next_edge_id, curGP.degree));
		else //GO_FORWARD
			guide.actions.push_back(setAction(curGP.to_node_id, curGP.cur_edge_id, 0));
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
	Node* nextnode = m_map.findNode(curGP.to_node_id);
	if (nextnode != nullptr)
		guide.heading_node_id = nextnode->id;

	//make string msg
	guide.distance_to_remain = m_rmdistance;

	//make guidance string
	guide.msg = getStringGuidance(guide, m_mvstatus);

	m_curguidance = guide;

	return true;
}

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

bool GuidanceManager::setArrivalGuide()
{
	Guidance guide;
	GuidedPathElement lastguide = m_guides.back();
	Node* dest = m_map.findNode(lastguide.from_node_id);
	if (dest == nullptr)
	{
		setEmptyGuide();
		return false;
	}
	
	//if last turn exists,
	if (!isForward(lastguide.degree))
	{
		Motion cmd = getMotion(dest->type, Edge::EDGE_SIDEWALK, lastguide.degree);
		Mode mode = getMode(Edge::EDGE_SIDEWALK);
		Action action = setActionCmd(cmd, Edge::EDGE_SIDEWALK, lastguide.degree, mode);
		guide.actions.push_back(action);
		guide.msg = getStringTurn(action, dest->type) + " and ";
	}

	guide.guide_status = GuideStatus::GUIDE_ARRIVED;
	guide.actions.push_back(setActionCmd(Motion::STOP,
		Edge::EDGE_SIDEWALK, 0, Mode::MOVE_NORMAL));
	guide.distance_to_remain = 0;
	guide.msg = guide.msg + " Arrived!";
	m_curguidance = guide;
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
	setPathNMap(path, m_map);

	//restart with index 0
	if (!initializeGuides()) return false;
	return true;
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

GuidanceManager::Action GuidanceManager::addTurnAction(GuidedPathElement gp)
{
	Action act;

	Edge* nextEdge = m_map.findEdge(gp.next_edge_id);
	if (nextEdge == nullptr)
	{
		printf("[Error] GuidanceManager::setNormalGuide] Cannot find edge id!\n");
		Node* node = m_map.findNode(gp.from_node_id);
		Motion cmd = getMotion(node->type, Edge::EDGE_SIDEWALK, gp.degree);
		act = setActionCmd(cmd, Edge::EDGE_SIDEWALK, gp.degree, Mode::MOVE_NORMAL);
	}
	else
	{
		act = setAction(gp.to_node_id, gp.next_edge_id, gp.degree);
	}

	return act;
}

GuidanceManager::Action GuidanceManager::setActionCmd(Motion cmd, int etype, int degree, Mode mode)
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
	result = setActionCmd(cmd, edge->type, degree, mode);
	return result;
}

std::string GuidanceManager::getStringForward(Action act, int ntype, ID nid, double d)
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

std::string GuidanceManager::getStringTurn(Action act, int ntype)
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

std::string GuidanceManager::getStringTurnDist(Action act, int ntype, double dist)
{
	std::string result;

	std::string motion = m_motions[(int)act.cmd];
	std::string edge = m_edges[act.edge_type];
	std::string degree = std::to_string(act.degree);
	std::string distance = std::to_string(dist);
	std::string str_act = "After " + distance + "m " + motion + " for " + degree + " degree";

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
			//str_first = getStringTurn(actions[i], node->type);
			str_first = getStringTurnDist(actions[i], node->type, guidance.distance_to_remain);
		}
		str.push_back(str_first);
	}

	//result = "[Guide] " + str[0];
	
	result = "[Guide] [" + m_mvstring[(int) status] + "]" + str[0];
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

int GuidanceManager::getGuideIdxFromPose(TopometricPose pose)
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

bool GuidanceManager::setPathNMap(dg::Path path, dg::Map map)
{
	if (path.pts.size() < 1)
	{
		printf("No path input!\n");
		return false;
	}
	if (!validatePath(path, map))
	{
		printf("Path id is not in map!\n");
		return false;
	}
	m_path = path;
	m_map = map;
	return true;
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

	double rad2deg = rad / 3.14 * 180.0;
	if (sign < 0.f) rad2deg = -rad2deg;	//this is for -180~180

	int result = (int) rad2deg;

	return result;

}

bool GuidanceManager::setEmptyGuide()
{
	Guidance guide;
	guide.actions[0].cmd = Motion::STOP;
	guide.msg = "No info\n";
	return true;
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

