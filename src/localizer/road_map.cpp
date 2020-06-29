#include "road_map.hpp"

#define ROAD_MAP_BUF_SIZE               (1024)

namespace dg
{

// 'RoadMap' class
bool RoadMap::load(const char* filename)
{
    removeAll();

    FILE* fid = fopen(filename, "rt");
    if (fid == nullptr) return false;

    char buffer[ROAD_MAP_BUF_SIZE];
    while (!feof(fid))
    {
        if (fgets(buffer, ROAD_MAP_BUF_SIZE, fid) == nullptr) break;
        char* token;
        if ((token = strtok(buffer, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        if (token[0] == 'N' || token[0] == 'n')
        {
            // Read nodes
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double x = strtod(token, nullptr);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double y = strtod(token, nullptr);

            if (addNode(Point2ID(id, x, y)) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        }
        else if (token[0] == 'E' || token[0] == 'e')
        {
            // Read edges
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id1 = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id2 = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double cost = strtod(token, nullptr);
            Node* node1 = getNode(Point2ID(id1));
            Node* node2 = getNode(Point2ID(id2));
            if (node1 == nullptr || node2 == nullptr) goto ROADMAP_LOADMAP_FAIL;

            if (cost < 0)
            {
                double dx = node2->data.x - node1->data.x;
                double dy = node2->data.y - node1->data.y;
                cost = sqrt(dx * dx + dy * dy);
            }
            if (addEdge(node1, node2, cost) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        }
    }
    fclose(fid);
    return true;

ROADMAP_LOADMAP_FAIL:
    removeAll();
    fclose(fid);
    return false;
}

bool RoadMap::save(const char* filename)
{
    if (isEmpty()) return false;

    FILE* file = fopen(filename, "wt");
    if (file == nullptr) return false;
    fprintf(file, "# NODE, ID, X [m], Y [m]\n");
    fprintf(file, "# EDGE, ID(from_ptr), ID(to_ptr), Cost\n");

    // Write nodes
    RoadMap::NodeItr itr_node;
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
        fprintf(file, "NODE, %zd, %lf, %lf\n", itr_node->data.id, itr_node->data.x, itr_node->data.y);

    // Write edges
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
    {
        RoadMap::EdgeItr itr_edge;
        for (itr_edge = getHeadEdge(itr_node); itr_edge != getTailEdge(itr_node); itr_edge++)
            fprintf(file, "EDGE, %zd, %zd, %lf\n", itr_node->data.id, itr_edge->to->data.id, itr_edge->cost);
    }

    fclose(file);
    return true;
}

bool RoadMap::addRoad(Node* node1, Node* node2, double cost /*= -1.0*/)
{
    if (node1 == nullptr || node2 == nullptr) return false;

    if (cost < 0)
    {
        double dx = node1->data.x - node2->data.x;
        double dy = node1->data.y - node2->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    Edge* edge1 = DirectedGraph<Point2ID, double>::addEdge(node1, node2, cost);
    Edge* edge2 = DirectedGraph<Point2ID, double>::addEdge(node2, node1, cost);
    return (edge1 != nullptr) && (edge2 != nullptr);
}

RoadMap::Edge* RoadMap::addEdge(Node* from, Node* to, double cost /*= -1.0*/)
{
    if (from == nullptr || to == nullptr) return nullptr;

    if (cost < 0)
    {
        double dx = from->data.x - to->data.x;
        double dy = from->data.y - to->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    return DirectedGraph<Point2ID, double>::addEdge(from, to, cost);
}

RoadMap::Edge* RoadMap::getEdge(ID from, ID to)
{
    Node* fnode = getNode(from);
    if (fnode == nullptr) return nullptr;
    for (EdgeItr edge_itr = getHeadEdge(fnode); edge_itr != getTailEdge(fnode); edge_itr++)
        if (edge_itr->to->data.id == to) return &(*edge_itr);
    return nullptr;
}

RoadMap::Edge* RoadMap::getEdge(Node* from, int edge_idx)
{
    if (from == nullptr) return nullptr;
    int edge_cnt = 0;
    for (auto edge_itr = getHeadEdge(from); edge_itr != getTailEdge(from); edge_itr++, edge_cnt++)
        if (edge_cnt == edge_idx) return &(*edge_itr);
    return nullptr;
}

bool RoadMap::copyTo(RoadMap* dest) const
{
    if (dest == nullptr) return false;

    dest->removeAll();
    for (NodeItrConst node = getHeadNodeConst(); node != getTailNodeConst(); node++)
        dest->addNode(node->data);
    for (NodeItrConst from = getHeadNodeConst(); from != getTailNodeConst(); from++)
        for (EdgeItrConst edge = getHeadEdgeConst(from); edge != getTailEdgeConst(from); edge++)
            dest->addEdge(dest->getNode(from->data), dest->getNode(edge->to->data), edge->cost);
    return true;
}

} // End of 'dg'
