#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

bool dg::Guidance::initializeGuides()
{
	if (m_map.nodes.empty() || m_path.pts.size() < 1)
		return false;

	std::vector<PathElement>::iterator iter = m_path.pts.begin();

	m_guides.clear();
	//m_pathNodeIds.clear();
	dg::PathElement dest = m_path.pts.back();

	dg::PathElement tempElement;
	//m_pathNodeIds.push_back(curnodeid);
	while (iter != m_path.pts.end())
	{
		tempElement = *iter;
		dg::Node* curNode = tempElement.node;
		
		iter++;
		tempElement = *iter;
		dg::Edge* curEdge = tempElement.edge;

		iter++;
		tempElement = *iter;
		dg::Node* nextNode = tempElement.node;

		//validate connection between nodes
		//if (nextNode == NULL)  

		//1. Find type of next node
		//[Constraints] only pedestrian is connected
		int navnodetype, dgnodetype;
		navnodetype = nextNode->type;
		/*
		// In case both are different
		switch (nextNode->data.type)
		{
		case 0:
			navnodetype = NodeType.POI;
			break;
		case 1:
			navnodetype = NodeType.JUNCTION;
			break;
		default:
			break;
		}
		*/

		//type compare with naver and jsh
		//current path file misses some edges.(2020-01-14)		
		size_t nEdges = nextNode->edge_list.size();
		if (nEdges <= 1)
		{
			//The node is end. Need to cross road. Find next node from the map.path
			dgnodetype = NT_JT;	// dgnodetype = ROAD_END;
			//dgnodetype = NodeEnum::NT_JT;	// dgnodetype = ROAD_END;
		}
		else if (nEdges <= 2)
		{
			//Continuous sidewalk, streets are connected with 180 degree
			dgnodetype = NT_BS;	//dgnodetype = POI;
		}
		else
		{
			//Junction, streets are connected with 90 DEGREE
			dgnodetype = NT_JT;	//dgnodetype = JUNCTION;
		}

		//2. Find edge connection to derive edgetype
		int dgedgetype;
		if (curEdge == NULL) //if two nodes are not linked, road.
		{
			dgedgetype = ET_MD;	//dgedgetype = ROAD;
		}
		else if (curNode->type == 1 && nextNode->type == 1) //if types of connected two nodes are 1, crosswalk.
		{
			dgedgetype = ET_CR;		//dgedgetype = CROSSWALK; //crosswalk is not identified in the Edge's type(2020-01-14, Check with naver)
		}
		else
		{
			dgedgetype = ET_SD;	//dgedgetype = SIDEWALK;
		}
		//if (*iter != NULL) //if two nodes are not linked, road.
		
		//3. calculate degree
		int angle;
		dg::Node* afterNextNode;
		if (iter == m_path.pts.end())	//last node check
		{
			iter++;	//next edge id
			iter++;
			if (iter == m_path.pts.end())	//last node check
			{
				angle = 0;
				break;
			}

			tempElement = *iter;
			dg::Node* afterNextNode = tempElement.node;
			angle = getDegree(curNode, nextNode, afterNextNode);

		}
		else
		{
			angle = 0;
		}

		GuidedPathElement tmppath(curNode, curEdge, nextNode, angle);
		m_guides.push_back(tmppath);

		iter--;
		iter--;

	}//while (curnode != dest_node)
	
	GuidedPathElement tmppath(dest.node, nullptr, nullptr, 0);
	m_guides.push_back(tmppath);

	return true;
}


dg::Guidance::MoveStatus dg::Guidance::applyPose(dg::TopometricPose  pose)
{
	//Current robot location
	dg::ID nodeid = pose.node_id;
	int eidx = pose.edge_idx;
	dg::Node* curnode = m_map.findNode(nodeid);
	dg::Edge* curedge = curnode->edge_list[eidx];
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
		m_mvstatus = MoveStatus::ARRIVED_NODE;
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


std::vector<dg::Guidance::RobotGuide> dg::Guidance::getNormalGuide(MoveStatus status)
{	
	std::vector<RobotGuide> result;
	GuidedPathElement curGP = m_guides[m_guide_idx];
	GuideStruct curG = setGuide(curGP.to_node->id, curGP.to_node->type, curGP.edge->type, curGP.degree);
	GuidedPathElement nextGP = m_guides[m_guide_idx +1];
	GuideStruct nextG = setGuide(nextGP.to_node->id, nextGP.to_node->type, nextGP.edge->type, nextGP.degree);

	ActionStruct act;
	RobotGuide rg;
	switch (status)
	{
	case MoveStatus::ON_EDGE: //maintain current guide
		result.push_back(m_prev_rguide.back());
		break;

	case MoveStatus::APPROACHING_NODE: //add next action
		result.push_back(m_prev_rguide.back());
		if (curGP.to_node->type == NT_JT)
		{
			act = setAction(Motion::STOP, 0);
			rg = setRobotGuide(curG, act);
			result.push_back(rg);
		}		
		break;

	case MoveStatus::ARRIVED_NODE: //add next action		
		if (45 < nextGP.degree && nextGP.degree <= 315)
		{
			act = setAction(Motion::TURN, curGP.degree);
			rg = setRobotGuide(curG, act);
			result.push_back(rg);
			
			act = setAction(Motion::STOP, 0);
			rg = setRobotGuide(curG, act);
			result.push_back(rg);
		}

		act = setAction(Motion::GO_FORWARD, 0);
		rg = setRobotGuide(nextG, act);
		result.push_back(rg);
		
		m_guide_idx++;

		//finishing condition
		if (m_guide_idx == m_guides.size())
			return result;
		break;

	default:
		break;
	}

	m_prev_rguide = result;
	
	return result;
}


void dg::Guidance::printSingleRGuide(dg::Guidance::RobotGuide instGuide)
{
	std::string nodetype;
	switch (instGuide.guide.node_type)
	{
	case NT_BS:
		nodetype = "POI";
		break;
	case NT_JT:
		nodetype = "JUNCTION";
		break;
	default:
		nodetype = "Unknown";
		break;
	}
	std::string rotation;
	if (315 < instGuide.action.degree || instGuide.action.degree <= 45)
		rotation = "STRAIGHT"; //0 degree
	else if (45 < instGuide.action.degree && instGuide.action.degree <= 135)
		rotation = "LEFT";
	else if (135 < instGuide.action.degree && instGuide.action.degree <= 225)
		rotation = "BACK";
	else if (225 < instGuide.action.degree && instGuide.action.degree <= 315)
		rotation = "RIGHT";
	else
		rotation = "STRAIGHT";

	std::string id = std::to_string(instGuide.guide.node_id);

	std::string edge;
	switch (instGuide.guide.edge_type)
	{
	case ET_SD:
		edge = "SIDEWALK";
		break;
	case ET_CR:
		edge = "CROSSWALK";
		break;
	case ET_MD:
		edge = "ROAD";
		break;
	default:
		edge = "Unknown";
		break;
	}

	std::string move;
	std::string result;
	switch (instGuide.action.move)
	{
	case Motion::GO_FORWARD:
		move = "GO_FORWARD on ";
		result = "[Guide] " + move + edge + " until next " + nodetype + "(Node ID : " + id + ")";
		break;
	case Motion::STOP:
		move = "STOP at ";
		result = "[Guide] " + move + nodetype + "(Node ID : " + id + ")";
		break;
	case Motion::TURN:
		move = "TURN ";
		result = "[Guide] " + move + rotation + " on " + nodetype + " (Node ID : " + id + ")";
		break;
	default:
		move = "Unknown";
		result = "[Guide] " + move + edge + " until next " + nodetype + "(Node ID : " + id + ")";
		break;
	}

	fprintf(stdout, "%s\n", result.c_str());

	return;
}

void dg::Guidance::printRobotGuide(std::vector<dg::Guidance::RobotGuide> rGuides)
{
	if (rGuides.empty())	return;
	
	for (size_t j = 0; j < rGuides.size(); j++)
	{
		printSingleRGuide(rGuides[j]);
	}
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
	int dg::Guidance::getDegree(double x1, double y1, double x2, double y2, double x3, double y3)

	dg::Map::Node* foundNode1 = map.findNode(559542564800587);
	dg::Map::Node* foundNode2 = map.findNode(559542564800586);
	dg::Map::Node* foundNode3 = map.findNode(559542564800055);
	dg::Map::Node* foundNode4 = map.findNode(559542564800056);
	dg::Map::Node* foundNode5 = map.findNode(559552564800620);

	int angle = m_guider.getDegree(&foundNode1->data, &foundNode2->data, &foundNode3->data);
	printf("Node: %llu --> Angle is: %d\n", foundNode1->data.id, angle);
*/
int dg::Guidance::getDegree(dg::Node* node1, dg::Node* node2, dg::Node* node3)
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


bool dg::Guidance::loadLocFiles(const char* filename, std::vector<dg::TopometricPose>& destination)
{
	destination.clear();

	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line;
	char data[1024];
	char* token;
	while (std::getline(is, line))
	{
		strcpy(data, line.c_str());
		token = strtok(data, "(,)");
		ID nodeid = (ID)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		ID edgeid = (ID)atoll(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		double dist = strtod(token, NULL);

		dg::TopometricPose pose = dg::TopometricPose((ID)nodeid, (ID)edgeid, dist);
		destination.push_back(pose);
	}

	is.close();
	return true;

GUIDANCE_LOADPATH_FAIL:
	destination.clear();
	is.close();
	return false;

}
