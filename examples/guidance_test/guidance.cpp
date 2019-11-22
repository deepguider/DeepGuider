#include "guidance.hpp"
#include "dg_map_manager.hpp"
#include "../unit_test/vvs.h"

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
		if (!nextG.direction == 0)
		{
			result.push_back(InstantGuide(curG, Action(TURN, nextG.direction)));
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
