#include "guidance.hpp"
#include "dg_map_manager.hpp"

using namespace dg;

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
	
	if (nodefile == NULL) return -1;

	//dg::MapManager manager;
	std::list<dg::ID> pathnodeids;
	std::list<dg::ID> pathedgeids;

	std::list<dg::ID> pathids = m_pathnodeids;
	std::list<dg::ID>::iterator it = pathids.begin();
	dg::ID pastid, curid;
	pastid = *pathids.begin();
	dg::Map::Node* foundnode = m_map.findNode(pastid);
	fprintf(nodefile, "%" PRIu64 "\n", foundnode->data.id);
	pathids.pop_front(); //첫번째 노드 삭제
		
	for (it = pathids.begin(); it != pathids.end(); ++it)
	{
		curid = *it;
		dg::Map::Node* foundnode = m_map.findNode(curid);
		//dg::Map::Edge* foundedge = m_map.findEdge(pastid, curid); //<--impossible! currently nodes are not connected.(by JSH 2019-11-21)
		/*for (size_t i = 0; i < foundnode->data.edge_ids.size(); i++)
		{
			pathedgeids.push_back(foundnode->data.edge_ids[i]);
			fprintf(edgefile, "%" PRIu64 "\n", foundnode->data.edge_ids[i]);
		}*/
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
		PathType first = (PathType) atoi(token);
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

dg::Guidance::Status dg::Guidance::checkStatus(dg::TopometricPose  pose)
{
	//Current robot location
	dg::ID curnode = pose.node_id;
	dg::ID curedge = pose.edge_idx;
	double curdist = pose.dist;
	
	/**Check progress.
	m_progress: 0.0 ~ 1.0
	'm_progress' indicates where the robot is.
	It also works as deciding the boundary for searching area in case of lost.
	*/	
	int pathlen = m_path.size();
	float pastprog = m_progress;

	//check whether robot's location is on the path.
	int pathidx;	
	for (pathidx = 0; pathidx < m_path.size(); ++pathidx)
	{
		PathInfo tmppath = m_path[pathidx];
		if ((tmppath.component == NODE) && (tmppath.id == curnode))
		{			
			/**if the difference of current location and progress is under threshold, finish check.
			The threshold is 10%. 
			Checking edge is future work.
			*/
			if (abs(pathidx / pathlen - pastprog) < 0.1)	
			{
				//update progress
				m_progress = pathidx / pathlen + curdist / pathlen;
				m_curpathindicator = pathidx;
			}
			//else //if the difference is big, the robot maybe lost. Future work.
			break;
		}
	}


	//Check path following status
	if (curdist == 0)
	{
		m_status = ARRIVED_NODE;
	}
	else if(curdist < 0.9)
	{
		m_status = ON_EDGE;
	}
	else if (curdist >= 0.9)
	{
		m_status = APPROACHING_NODE;
	}

	return Status(m_status);
}


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
		
		/**guide: "until where" "in which direction" "with mode"
		related to "NodeType" "Direction" "EdgeType" 
		*/
		m_guide.push_back(Guide(nextnode.id, nextnode.node, curnode.degree, curedge.edge));
		curnode = nextnode;
	}

	m_guide.push_back(Guide(nextnode.id, nextnode.node, nextnode.degree, nextnode.edge));

	return true;;
}

bool dg::Guidance::generateGuidancePath(dg::Map& map, dg::Path path)
{
	dg::MapManager m_map_manager;
	std::list<dg::ID>::iterator iter = path.m_points.begin();

	m_path.clear();
	m_pathnodeids.clear();

	dg::ID start_node = path.m_points.front();
	dg::ID dest_node = path.m_points.back();

	dg::ID curnodeid = start_node;
	m_pathnodeids.push_back(curnodeid);
	iter++;
	while (curnodeid != dest_node)
	{
		dg::Map::Node* curNode = map.findNode(curnodeid);
		dg::ID nextnodeid = *iter;
		dg::Map::Node* nextNode = map.findNode(nextnodeid);
		if (nextNode == NULL) continue; //validate connection between nodes

		//1. Find type of next node
		//only pedestrian is connected
		dg::Guidance::NodeType navnodetype, dgnodetype;
		/*
		switch (nextNode->data.type)
		{
		case 0:
			navnodetype = POI;
			break;
		case 1:
			navnodetype = JUNCTION;
			break;
		default:
			break;
		}
		*/

		//type compare with naver and jsh
		//current path file misses some edges.(2020-01-14)		
		int nEdges = map.countEdges(nextNode);
		if (nEdges <= 1)
		{
			//The node is end. Need to cross road. Find next node from the map.path
			dgnodetype = ROAD_END;
		}
		else if (nEdges <= 2)
		{
			//Continuous sidewalk, streets are connected with 180 degree
			dgnodetype = POI;
		}
		else
		{
			//Junction, streets are connected with 90 DEGREE
			dgnodetype = JUNCTION;
		}

		//2. Find edge connection to derive edgetype
		dg::Guidance::EdgeType dgedgetype;
		dg::Map::Edge* curEdge = map.findEdge(curnodeid, nextnodeid);
		if (curEdge == NULL) //if two nodes are not linked, road.
		{
			dgedgetype = ROAD;
		}
		else if (curNode->data.type == 1 && nextNode->data.type == 1) //if types of connected two nodes are 1, crosswalk.
		{
			dgedgetype = CROSSWALK; //crosswalk is not identified in the Edge's type(2020-01-14, Check with naver)
		}
		else
		{
			dgedgetype = SIDEWALK;
		}

		//3. calculate degree
		int angle;
		if (*iter != dest_node)
		{
			iter++;
			dg::ID afterNextNodeId = *iter;
			dg::Map::Node* afterNectNode = map.findNode(afterNextNodeId);
			if (afterNectNode == NULL) continue;
			angle = getDegree(&curNode->data, &nextNode->data, &afterNectNode->data);			
		}
		else
		{
			angle = 0;
		}
		//Add node to guidance
		dg::Guidance::PathInfo nodepath(NODE, curNode->data.id, dgnodetype, angle);
		m_path.push_back(nodepath);
		m_pathnodeids.push_back(curnodeid);

		//Add guidance of this edge
		dg::Guidance::PathInfo edgepath(EDGE, 00000000, dgedgetype);
		m_path.push_back(edgepath);

		curnodeid = nextnodeid;

	}//while (curnode != dest_node)

	//Add last node to guidance
	dg::Map::Node* curNode = map.findNode(curnodeid);
	dg::Guidance::PathInfo nodepath(NODE, curNode->data.id, POI, 0);
	m_path.push_back(nodepath);
	m_pathnodeids.push_back(curnodeid);

	return true;
}


std::vector<dg::Guidance::InstantGuide> dg::Guidance::provideNormalGuide(std::vector<InstantGuide> prevguide, Status status)
{	
	std::vector<InstantGuide> result;
	int Gidx = m_curguideindicator;
	Guide curG = m_guide[Gidx]; 
	Guide nextG = m_guide[Gidx + 1];	

	switch (status)
	{
	case ON_EDGE: //maintain current guide
		result.push_back(prevguide.back());		
		break;

	case APPROACHING_NODE: //add next action
		result.push_back(prevguide.back());
		if (curG.type == NodeType::JUNCTION)
		{
			result.push_back(InstantGuide(curG, Action(STOP, 0)));
		}		
		break;

	case ARRIVED_NODE: //add next action		
		if (45 < nextG.degree && nextG.degree <= 315)
		{
			result.push_back(InstantGuide(curG, Action(TURN, nextG.degree)));
			result.push_back(InstantGuide(curG, Action(STOP, 0)));
		}
		
		result.push_back(InstantGuide(nextG, Action(GO_FORWARD, 0)));
		
		m_curguideindicator++;
		//finishing condition
		if (m_curguideindicator == m_guide.size())
			return result;
		break;

	default:
		break;
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

void dg::Guidance::printInstantGuide(dg::Guidance::InstantGuide instGuide)
{

//	char nodetype[10];
	std::string nodetype;
	switch (instGuide.guide.type)
	{
	case POI:
		nodetype = "POI";
		break;
	case JUNCTION:
		nodetype = "JUNCTION";
		break;
	case ROAD_END:
		nodetype = "ROAD_END";
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
	switch (instGuide.guide.mode)
	{
	case SIDEWALK:
		edge = "SIDEWALK";
		break;
	case CROSSWALK:
		edge = "CROSSWALK";
		break;
	case ROAD:
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
	case GO_FORWARD:
		move = "GO_FORWARD on ";
		result = "[Guide] " + move + edge + " until next " + nodetype + "(Node ID : " + id + ")" ;
		break;
	case STOP:
		move = "STOP at ";
		result = "[Guide] " + move + nodetype + "(Node ID : " + id + ")";
		break;
	case TURN:
		move = "TURN on ";
		result = "[Guide] " + move + nodetype + "(Node ID : " + id + ")," + " in " + rotation + " direction";
		break;
	default:
		move = "Unknown";
		result = "[Guide] " + move + edge + " until next " + nodetype + "(Node ID : " + id + ")";
		break;
	}

	fprintf(stdout, "%s\n", result.c_str());

	return;
}