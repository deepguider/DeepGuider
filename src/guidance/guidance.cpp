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
	for (pathidx = 1; pathidx < m_path.size(); ++pathidx)
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
		result.push_back(InstantGuide(curG, Action(STOP, 0)));
		break;

	case ARRIVED_NODE: //add next action		
		if (!nextG.degree == 0)
		{
			result.push_back(InstantGuide(curG, Action(TURN, nextG.degree)));
			result.push_back(InstantGuide(curG, Action(STOP, 0)));
		}

		m_curguideindicator++;
		//finishing condition
		if (m_curguideindicator == m_guide.size() - 1)
			return result;

		result.push_back(InstantGuide(nextG, Action(GO_FORWARD, 0)));
		break;

	default:
		break;
	}

	
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
	case CORNER:
		nodetype = "CORNER";
		break;
	case ISLAND:
		nodetype = "ISLAND";
		break;
	default:
		nodetype = "Unknown";
		break;
	}

//	char edge[10];
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
	switch (instGuide.action.move)
	{
	case GO_FORWARD:
		move = "GO_FORWARD on ";
		break;
	case STOP:
		move = "STOP at ";
		break;
	case TURN:
		move = "TURN at ";
		break;
	default:
		move = "Unknown";
		break;
	}

	std::string id = std::to_string(instGuide.guide.nodeid);

	std::string deg = std::to_string(instGuide.action.degree);
	
	std::string result = "[Guide] Until next " + nodetype + "(Node ID: " + id + "), " + move + edge + " in " + deg + " degree.";
	fprintf(stdout, "%s\n", result.c_str());

	return;
}