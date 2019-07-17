#include "simple_road_map.hpp"

#define ROAD_MAP_BUF_SIZE               (1024)

namespace dg
{

// 'SimpleRoadMap' class
SimpleRoadMap::SimpleRoadMap()
{
}

bool SimpleRoadMap::load(const char* filename)
{
    removeAll();

    FILE* fid = fopen(filename, "rt");
    if (fid == NULL) return false;

    char buffer[ROAD_MAP_BUF_SIZE];
    while (!feof(fid))
    {
        if (fgets(buffer, ROAD_MAP_BUF_SIZE, fid) == NULL) break;
        char* token;
        if ((token = strtok(buffer, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
        if (token[0] == 'N' || token[0] == 'n')
        {
            // Read nodes
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            int id = strtol(token, NULL, 10);
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            double x = strtod(token, NULL);
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            double y = strtod(token, NULL);

            if (addNode(Point2ID(id, x, y)) == NULL) goto ROADMAP_LOADMAP_FAIL;
        }
        else if (token[0] == 'E' || token[0] == 'e')
        {
            // Read edges
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            int id1 = strtol(token, NULL, 10);
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            int id2 = strtol(token, NULL, 10);
            if ((token = strtok(NULL, ",")) == NULL) goto ROADMAP_LOADMAP_FAIL;
            double cost = strtod(token, NULL);
            Node* node1 = getNode(Point2ID(id1));
            Node* node2 = getNode(Point2ID(id2));
            if (node1 == NULL || node2 == NULL) goto ROADMAP_LOADMAP_FAIL;

            if (cost < 0)
            {
                double dx = node2->data.x - node1->data.x;
                double dy = node2->data.y - node1->data.y;
                cost = sqrt(dx * dx + dy * dy);
            }
            if (addEdge(node1, node2, cost) == NULL) goto ROADMAP_LOADMAP_FAIL;
        }
    }
    fclose(fid);
    return true;

ROADMAP_LOADMAP_FAIL:
    removeAll();
    fclose(fid);
    return false;
}

bool SimpleRoadMap::save(const char* filename)
{
    if (isEmpty()) return false;

    FILE* file = fopen(filename, "wt");
    if (file == NULL) return false;
    fprintf(file, "# NODE, ID, X [m], Y [m]\n");
    fprintf(file, "# EDGE, ID(from), ID(to), Cost\n");

    // Write nodes
    SimpleRoadMap::NodeItr itr_node;
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
        fprintf(file, "NODE, %d, %lf, %lf\n", itr_node->data.id, itr_node->data.x, itr_node->data.y);

    // Write edges
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
    {
        SimpleRoadMap::EdgeItr itr_edge;
        for (itr_edge = getHeadEdge(itr_node); itr_edge != getTailEdge(itr_node); itr_edge++)
            fprintf(file, "EDGE, %d, %d, %lf\n", itr_node->data.id, itr_edge->to->data.id, itr_edge->cost);
    }

    fclose(file);
    return true;
}

bool SimpleRoadMap::isEmpty() const
{
    return (countNodes() <= 0);
}

bool SimpleRoadMap::addRoad(Node* node1, Node* node2, double cost /*= -1.0*/)
{
    if (node1 == NULL || node2 == NULL) return false;

    if (cost < 0)
    {
        double dx = node1->data.x - node2->data.x;
        double dy = node1->data.y - node2->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    Edge* edge1 = DirectedGraph<Point2ID, double>::addEdge(node1, node2, cost);
    Edge* edge2 = DirectedGraph<Point2ID, double>::addEdge(node2, node1, cost);
    return (edge1 != NULL) && (edge2 != NULL);
}

bool SimpleRoadMap::addRoad(const Point2ID& node1, const Point2ID& node2, double cost /*= -1.0*/)
{
    if (cost < 0)
    {
        double dx = node1.x - node2.x;
        double dy = node1.y - node2.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    Edge* edge1 = DirectedGraph<Point2ID, double>::addEdge(node1, node2, cost);
    Edge* edge2 = DirectedGraph<Point2ID, double>::addEdge(node2, node1, cost);
    return (edge1 != NULL) && (edge2 != NULL);
}

SimpleRoadMap::Edge* SimpleRoadMap::addEdge(Node* from, Node* to, double cost /*= -1.0*/)
{
    if (from == NULL || to == NULL) return false;

    if (cost < 0)
    {
        double dx = from->data.x - to->data.x;
        double dy = from->data.y - to->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    return DirectedGraph<Point2ID, double>::addEdge(from, to, cost);
}

SimpleRoadMap::Edge* SimpleRoadMap::addEdge(const Point2ID& from, const Point2ID& to, double cost /*= -1.0*/)
{
    if (cost < 0)
    {
        double dx = from.x - to.x;
        double dy = from.y - to.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    return DirectedGraph<Point2ID, double>::addEdge(from, to, cost);
}

} // End of 'dg'
