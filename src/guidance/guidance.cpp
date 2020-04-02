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
		Edge* nextEdge = tempElement.edge;

		GuidedPathElement tmppath(curNode, curEdge, nextNode, nextEdge, angle);
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
	GuidedPathElement tmppath(dest.node, nullptr, nullptr, nullptr, 0);
	m_guides.push_back(tmppath);

	m_past_guides.clear();
	Guidance guide = getGuidance(MoveStatus::ON_NODE);

	return true;

}


GuidanceManager::MoveStatus GuidanceManager::applyPose(TopometricPose  pose)
{
	//Current robot location
	ID nodeid = pose.node_id;
	int eidx = pose.edge_idx;
	Node* curnode = findNodeFromPath(nodeid);
	//	Node* curnode = m_map.findNode(nodeid);
	Edge* curedge = curnode->edge_list[eidx];
	double edgedist = curedge->length;
	double curdist = pose.dist;
	//double progress = curdist;
	double progress = curdist / edgedist; //will be exchanged

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
	GuidanceManager::Motion motion;
	std::string rotation;

	if (degree >= -45 && degree <= 45)	//"FORWARD"
	{		
		if (ntype == Node::NODE_BASIC && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::GO_FORWARD;
		else if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_FORWARD;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_FORWARD;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::EXIT_FORWARD;
	}
	else if (degree > 45 && degree <= 135)	//"LEFT"
	{		
		if (ntype == Node::NODE_BASIC && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::TURN_LEFT;
		else if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_LEFT;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_LEFT;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::EXIT_LEFT;
	}
	else if (degree < -45 && degree >= -135)	//"RIGHT"
	{
		if (ntype == Node::NODE_BASIC && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::TURN_RIGHT;
		else if (ntype == Node::NODE_JUNCTION && etype == Edge::EDGE_CROSSWALK)
			motion = GuidanceManager::Motion::CROSS_RIGHT;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::ENTER_RIGHT;
		else if (ntype == Node::NODE_DOOR && etype == Edge::EDGE_SIDEWALK)
			motion = GuidanceManager::Motion::EXIT_RIGHT;
	}
	else //"BACK"
		motion = GuidanceManager::Motion::TURN_BACK;

	return motion;	
}

GuidanceManager::Action GuidanceManager::setAction(GuidedPathElement gpe, int degree)
{
	Action result;	
	result.cmd = getMotion(gpe.to_node->type, gpe.cur_edge->type, degree);
	result.node_type = gpe.to_node->type;
	result.edge_type = gpe.cur_edge->type;
	result.degree = gpe.degree;
	result.mode = getMode(gpe.cur_edge->type);
	return result;
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


GuidanceManager::Guidance GuidanceManager::getGuidance(MoveStatus status)
{
	Guidance guide;
	GuidedPathElement curGP, nextGP;

	if (m_guide_idx == m_guides.size() - 1) goto GET_LAST_GUIDE;	

	//update dgstatus
	guide.guide_status = m_dgstatus;

	//update moving status
	guide.moving_status = status;

	//set actions
	switch (status)
	{
	case dg::GuidanceManager::MoveStatus::ON_NODE: 		//add next action
	{
		//if past degree is turn, add former turn on current node
		if (!m_past_guides.empty())
		{
			GuidedPathElement pastGP = getCurGuidedPath(m_guide_idx);	//from last guide
			if (!isForward(pastGP.degree))
			{
				Motion ncmd = getMotion(pastGP.to_node->type, pastGP.next_edge->type, pastGP.degree);
				Mode nmode = getMode(pastGP.next_edge->type);
				guide.actions.push_back(setAction(ncmd, pastGP.next_edge->type, pastGP.degree, nmode));
			}
		}

		//add go on current node
		int gidx = m_guide_idx + 1;
		curGP = getCurGuidedPath(gidx);
		if (gidx == m_guides.size() - 1) goto GET_LAST_GUIDE;
		guide.actions.push_back(setAction(curGP));

		//add final turn on next node
		if (!isForward(curGP.degree))
		{
			Motion ncmd = getMotion(curGP.to_node->type, curGP.next_edge->type, curGP.degree);
			Mode nmode = getMode(curGP.next_edge->type);
			guide.actions.push_back(setAction(ncmd, curGP.next_edge->type, curGP.degree, nmode));
		}

		//save guidances
		m_past_guides.push_back(guide);
		m_guide_idx++;

		break;
	}
	case dg::GuidanceManager::MoveStatus::ON_EDGE://maintain current guide
		//only forward action is extracted on edge
		curGP = getCurGuidedPath(m_guide_idx);
		guide.actions.push_back(setAction(curGP));
		break;
	case dg::GuidanceManager::MoveStatus::APPROACHING_NODE://add next action
	{
		//GO_FORWARD
		curGP = getCurGuidedPath(m_guide_idx);
		guide.actions.push_back(setAction(curGP));

		//if TURN exists on next node,
		if (!isForward(curGP.degree))
		{
			Motion ncmd = getMotion(curGP.to_node->type, curGP.next_edge->type, curGP.degree);
			Mode nmode = getMode(curGP.next_edge->type);
			guide.actions.push_back(setAction(ncmd, curGP.next_edge->type, curGP.degree, nmode));
		}
		break;
	}
	default:
		break;
	}

	//update heading_node
	guide.heading_node = *curGP.to_node;

	//update distance_to_remain
	guide.distance_to_remain = curGP.cur_edge->length * (1.0 - m_edge_progress);

	//make guidance string
	guide.msg = getGuidanceString(guide, status);

	return guide;

GET_LAST_GUIDE:
	guide = getLastGuidance();
	guide.msg = "Arrived!";
	return guide;
}


GuidanceManager::Guidance GuidanceManager::getGuidance(TopometricPose pose)
{
	MoveStatus curStatus = applyPose(pose);
	int gidx = getGuideIdxFromPose(pose);
	if (gidx != m_guide_idx)
	{
		m_guide_idx = gidx;
		if (pose.dist < 1.0)
		{
			curStatus = MoveStatus::ON_NODE;	//to update guide
		}
		else
		{
			curStatus = MoveStatus::ON_EDGE;	//to update guide
		}
	}
	Guidance guide = getGuidance(curStatus);
	return guide;
}

std::string dg::GuidanceManager::getForwardString(Action act, int ntype, ID nid, double d)
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

std::string dg::GuidanceManager::getTurnString(Action act, int ntype)
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

int dg::GuidanceManager::getGuideIdxFromPose(TopometricPose pose)
{	
	ID curnodei = pose.node_id;
	for (size_t i = 0; i < m_guides.size(); i++)
	{
		if (m_guides[i].from_node->id == curnodei)
		{
			return i;
		}		
	}
	return -1;
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

std::string GuidanceManager::getGuidanceString(Guidance guidance, MoveStatus status)
{
	std::string result, str_first;
	std::vector<std::string> str;
	std::vector<Action> actions = guidance.actions;

	int cnt = 0;
	for (size_t i = 0; i < actions.size(); i++)	//only 2 steps are shown in msg
	{
		if (isForward(actions[i].cmd))
		{
			str_first = getForwardString(actions[i], guidance.heading_node.type,
				guidance.heading_node.id, guidance.distance_to_remain);
		}
		else
		{
			str_first = getTurnString(actions[i], guidance.heading_node.type);
		}
		str.push_back(str_first);
	}

	result = "[Guide] " + str[0];
	if (actions.size() >= 2)
		result = result + " and " + str[1];

	return result;
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
