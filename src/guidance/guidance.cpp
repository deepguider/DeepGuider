#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

bool dg::Guidance::initializeGuides()
{
	if (m_map.isEmpty() || m_path.countPoints() < 1)
		return false;

	//dg::MapManager m_map_manager;
	std::list<dg::ID>::iterator iter = m_path.m_points.begin();

	m_initGuides.clear();
	//m_pathNodeIds.clear();

	dg::ID start_nodeId = m_path.m_points.front();
	dg::ID dest_nodeId = m_path.m_points.back();

	dg::ID curnodeid = start_nodeId;
	//m_pathNodeIds.push_back(curnodeid);
	while (curnodeid != dest_nodeId)
	{
		dg::Map::Node* curNode = m_map.findNode(curnodeid);

		iter++;
		dg::ID curedgeid = *iter;

		iter++;
		dg::ID nextnodeid = *iter;
		dg::Map::Node* nextNode = m_map.findNode(nextnodeid);
		//if (nextNode == NULL) continue; //validate connection between nodes

		//1. Find type of next node
		//[Constraints] only pedestrian is connected
		dg::Guidance::NodeEnum navnodetype, dgnodetype;
		navnodetype = (NodeEnum) nextNode->data.type;
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
		int nEdges = m_map.countEdges(nextNode);
		if (nEdges <= 1)
		{
			//The node is end. Need to cross road. Find next node from the map.path
			dgnodetype = NodeEnum::NT_JT;	// dgnodetype = ROAD_END;
		}
		else if (nEdges <= 2)
		{
			//Continuous sidewalk, streets are connected with 180 degree
			dgnodetype = NodeEnum::NT_BS;	//dgnodetype = POI;
		}
		else
		{
			//Junction, streets are connected with 90 DEGREE
			dgnodetype = NodeEnum::NT_JT;	//dgnodetype = JUNCTION;
		}

		//2. Find edge connection to derive edgetype
		dg::Guidance::EdgeEnum dgedgetype;

		//dg::Map::Edge* nextEdge = m_map.findEdge(curedgeid);
		dg::Map::Edge* curEdge = m_map.findEdge(curnodeid, nextnodeid);
		printf("\curedgeid=%llu\n", curedgeid);
		if (curEdge == NULL) //if two nodes are not linked, road.
		{
			dgedgetype = EdgeEnum::ET_MD;	//dgedgetype = ROAD;
		}
		else if (curNode->data.type == 1 && nextNode->data.type == 1) //if types of connected two nodes are 1, crosswalk.
		{
			dgedgetype = EdgeEnum::ET_CR;		//dgedgetype = CROSSWALK; //crosswalk is not identified in the Edge's type(2020-01-14, Check with naver)
		}
		else
		{
			dgedgetype = EdgeEnum::ET_SD;	//dgedgetype = SIDEWALK;
		}
		//if (*iter != NULL) //if two nodes are not linked, road.
		
		//3. calculate degree
		int angle;
		if (*iter != dest_nodeId)	//last node check
		{
			iter++;	//next edge id, 
			// dg::ID nextedgeid = *iter;
			iter++;
			dg::ID afterNextNodeId = *iter;
			if (afterNextNodeId == NULL)	//last node check
			{
				angle = 0;
				break;
			}
			dg::Map::Node* afterNextNode = m_map.findNode(afterNextNodeId);
			angle = getDegree(&curNode->data, &nextNode->data, &afterNextNode->data);

		}
		else
		{
			angle = 0;
		}

		//Add node to guidance
		//dg::Guidance::GuidedPathType nodePath(PathEnum::NODE, curNode->data.id, dgnodetype, angle);
		//m_initGuides.push_back(nodePath);
		//m_pathNodeIds.push_back(curnodeid);

		//Add guidance of this edge
		//dg::Guidance::GuidedPathType edgepath(PathEnum::EDGE, curedgeid, dgedgetype);
		//m_initGuides.push_back(edgepath);

		GuidedPathType tmppath(curnodeid, (NodeEnum)curNode->data.type, 
			curedgeid, dgedgetype, nextnodeid, (NodeEnum)nextNode->data.type, angle);
		m_initGuides.push_back(tmppath);

		curnodeid = nextnodeid;
		iter--;
		iter--;

	}//while (curnode != dest_node)

	//Add last node to guidance
	//dg::Guidance::GuidedPathType nodepath(PathEnum::NODE, dest_nodeId, NodeEnum::NT_BS, 0);
	//m_initGuides.push_back(nodepath);
	//m_pathNodeIds.push_back(dest_nodeId);		

	dg::Map::Node* destNode = m_map.findNode(dest_nodeId);
	GuidedPathType tmppath(dest_nodeId, (NodeEnum)destNode->data.type, (ID) 0, EdgeEnum::ET_SD, (ID)0, NodeEnum::NT_BS, 0);
	m_initGuides.push_back(tmppath);

	return true;
}


dg::Guidance::MoveStatus dg::Guidance::applyPose(dg::TopometricPose  pose)
{
	//Current robot location
	dg::ID nodeId = pose.node_id;
	dg::ID edgeId = pose.edge_idx;

	//if Edge distance
	double curdist = pose.dist;

	/**Check progress.
	m_edgeElapse: 0.0 ~ 1.0
	'm_edgeElapse' indicates where the robot on the edge is.
	It also works as deciding the boundary for searching area in case of lost.
	*/
	//size_t pathlen = m_initGuides.size();
	//double pastprog = m_edgeElapse;
	
	//int pathidx;
	//for (pathidx = 0; pathidx < m_initGuides.size(); ++pathidx)
	//{
	//	GuidedPathType tmpguide = m_initGuides[pathidx];
	//	if (tmpguide.id == curnode)
	//	{
	//		/**if the difference of current location and progress is under threshold, finish check.
	//		The threshold is 10%.
	//		Checking edge is future work.
	//		*/
	//		if (abs(pathidx / pathlen - pastprog) < 0.1)
	//		{
	//			//update progress
	//			m_edgeElapse = pathidx / pathlen + curdist / pathlen;
	//			m_curPathNodeIdx = pathidx;
	//		}
	//		//else //if the difference is big, the robot maybe lost. Future work.
	//		break;
	//	}
	//}

	GuidedPathType curGP = m_initGuides[m_curGuideIdx];

	//Check guide following status
	if ((nodeId != curGP.curnid) && (curdist == 0))
	{
		m_mvstatus = MoveStatus::ARRIVED_NODE;
	}
	else if (curdist >= 0.9)
	{
		m_mvstatus = MoveStatus::APPROACHING_NODE;
	}
	else
	{
		m_mvstatus = MoveStatus::ON_EDGE;
	}

	return MoveStatus(m_mvstatus);
}


std::vector<dg::Guidance::RobotGuide> dg::Guidance::getNormalGuide(MoveStatus status)
{	
	std::vector<RobotGuide> result;
	GuidedPathType curGP = m_initGuides[m_curGuideIdx];
	GuideStruct curG = setGuide(curGP.nextnid, curGP.nextntype, curGP.etype, curGP.degree);
	GuidedPathType nextGP = m_initGuides[m_curGuideIdx+1];
	GuideStruct nextG = setGuide(nextGP.nextnid, nextGP.nextntype, nextGP.etype, nextGP.degree);

	ActionStruct act;
	RobotGuide rg;
	switch (status)
	{
	case MoveStatus::ON_EDGE: //maintain current guide
		result.push_back(m_prevguide.back());
		break;

	case MoveStatus::APPROACHING_NODE: //add next action
		result.push_back(m_prevguide.back());
		if (curGP.nextntype == NodeEnum::NT_JT)
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
		
		m_curGuideIdx++;

		//finishing condition
		if (m_curGuideIdx == m_initGuides.size())
			return result;
		break;

	default:
		break;
	}

	m_prevguide = result;
	
	return result;
}


void dg::Guidance::printSingleRGuide(dg::Guidance::RobotGuide instGuide)
{

	//	char nodetype[10];
	std::string nodetype;
	switch (instGuide.guide.nodetype)
	{
	case NodeEnum::NT_BS:
		nodetype = "POI";
		break;
	case NodeEnum::NT_JT:
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

	std::string id = std::to_string(instGuide.guide.nodeid);

	std::string edge;
	switch (instGuide.guide.edgetype)
	{
	case EdgeEnum::ET_SD:
		edge = "SIDEWALK";
		break;
	case EdgeEnum::ET_CR:
		edge = "CROSSWALK";
		break;
	case EdgeEnum::ET_MD:
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
int dg::Guidance::getDegree(dg::NodeInfo* node1, dg::NodeInfo* node2, dg::NodeInfo* node3)
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

//for temporary use
/**
bool dg::Guidance::generateGuide()
{
	bool result = false;

	m_guide.clear();
	PathInfo curnode = m_path[0];
	PathInfo curedge(EDGE, 0, SIDEWALK);
	PathInfo nextnode(NODE, 0, POI, 0);

	int pathidx;
	for (pathidx = 0; pathidx < m_path.size(); pathidx++)
	{
		PathInfo tmppath = m_path[pathidx];
		if (tmppath.component == EDGE)
		{
			curedge = tmppath;
			continue;
		}
		else
		{
			nextnode = tmppath;
		}

		//guide: "until where" "in which direction" "with mode"
		//related to "NodeType" "Direction" "EdgeType"
		m_guide.push_back(Guide(nextnode.id, nextnode.node, curnode.degree, curedge.edge));
		curnode = nextnode;
	}

	m_guide.push_back(Guide(nextnode.id, nextnode.node, nextnode.degree, nextnode.edge));

	return true;;
}

bool dg::Guidance::validatePathNodeIds()
{
	std::list<dg::ID>::iterator it = m_pathnodeids.begin();
	dg::ID tempid;
	while (it != m_pathnodeids.end())
	{
		tempid = *it;
		dg::Map::Node* foundNode = m_map.findNode(tempid);
		if (foundNode == nullptr)
		{
			m_pathnodeids.remove(tempid);
			it = m_pathnodeids.begin();
		}
		else {
			++it;
		}
	}
	return false;
}

bool dg::Guidance::savePathNodeEdgeIds()
{
	FILE* nodefile = fopen("PathNodeIds.txt", "w");
	FILE* edgefile = fopen("PathEdgeIds.txt", "w");

	if (nodefile == NULL) return false;

	//dg::MapManager manager;
	std::list<dg::ID> pathnodeids;
	std::list<dg::ID> pathedgeids;

	std::list<dg::ID> pathids = m_pathnodeids;
	std::list<dg::ID>::iterator it = pathids.begin();
	dg::ID pastid, curid;
	pastid = *pathids.begin();
	dg::Map::Node* foundnode = m_map.findNode(pastid);
	fprintf(nodefile, "%" PRIu64 "\n", foundnode->data.id);
	pathids.pop_front(); //erase first node

	for (it = pathids.begin(); it != pathids.end(); ++it)
	{
		curid = *it;
		dg::Map::Node* foundnode = m_map.findNode(curid);
		//dg::Map::Edge* foundedge = m_map.findEdge(pastid, curid); //<--impossible! currently nodes are not connected.(by JSH 2019-11-21)
		//for (size_t i = 0; i < foundnode->data.edge_ids.size(); i++)
		//{
		//	pathedgeids.push_back(foundnode->data.edge_ids[i]);
		//	fprintf(edgefile, "%" PRIu64 "\n", foundnode->data.edge_ids[i]);
		//}
		pathnodeids.push_back(foundnode->data.id);
		fprintf(nodefile, "%" PRIu64 "\n", foundnode->data.id);

		pastid = curid;

	}

	m_pathedgeids = pathedgeids;

	fclose(nodefile);
	fclose(edgefile);

	return true;
}


bool dg::Guidance::loadPathFiles(const char* filename, std::list<ID>& destination)
{
	destination.clear();
	//destination.clear();
	auto is = std::ifstream(filename, std::ofstream::in);
	assert(is.is_open());
	std::string line, text;

	while (std::getline(is, line))
	{
		//text += line + "\n";				
		destination.push_back(stoull(line));
		//destination.push_back(stoull(line));
	}

	is.close();

	return false;
}


bool dg::Guidance::loadPathFiles(const char* filename, std::vector<PathInfo>& destination)
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
		PathType first = (PathType)atoi(token);
		if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
		ID id = (ID)atoll(token);
		if (first == 0)
		{
			if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
			NodeType type = (NodeType)atoi(token);
			if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
			int degree = atoi(token);
			PathInfo path = PathInfo((PathType)first, (ID)id, (NodeType)type, degree);
			destination.push_back(path);
		}
		else
		{
			if ((token = strtok(NULL, ",")) == NULL) goto GUIDANCE_LOADPATH_FAIL;
			EdgeType type = (EdgeType)atoi(token);
			PathInfo path = PathInfo((PathType)first, (ID)id, (EdgeType)type);
			destination.push_back(path);
		}

	}

	is.close();
	return true;

GUIDANCE_LOADPATH_FAIL:
	destination.clear();
	is.close();
	return false;
}
*/

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
