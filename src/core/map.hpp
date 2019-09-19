#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"
#include "core/directed_graph.hpp"
#include "core/graph_painter.hpp"

namespace dg
{

/**
 * @brief 2D point on the earth with ID
 *
 * A 2D point in the geodesic notation is defined with the identifier (shortly ID).
 * It is the counter part of LonLat, similar to Point2 and Point2ID.
 */
class LonLatID : public Point2ID
{
public:
    /**
     * A constructor with ID assignment
     * @param _id The givne ID
     */
    LonLatID(ID _id = 0) : Point2ID(_id), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param _x The given X
     * @param _y The given Y
     */
    LonLatID(ID _id, double _lon, double _lat) : Point2ID(_id, _lon, _lat), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param p The given 2D point
     */
    LonLatID(ID _id, const Point2& p) : Point2ID(_id, p), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param pid The given 2D point with ID
     */
    LonLatID(const Point2ID& pid) : Point2ID(pid), lon(x), lat(y) { }

    /** Latitude */
    double& lon;

    /** Latitude */
    double& lat;
};

class NodeInfo : public LonLatID
{
public:
    NodeInfo(ID _id = 0, double _lon = 0, double _lat = 0, int _type = 0, int _floor = 0) : LonLatID(_id, _lon, _lat), type(_type), floor(_floor) { }

    NodeInfo(ID _id, const LonLat& p, int _type = 0, int _floor = 0) : LonLatID(_id, p), type(_type), floor(_floor) { }

    /**
     * Overriding the assignment operator
     * @param rhs NodeInfo in the right-hand side
     * @return The assigned instance
     */
    NodeInfo& operator=(const NodeInfo& rhs)
    {
        id     = rhs.id;
        x      = rhs.x;
        y      = rhs.y;
        type   = rhs.type;
        floor  = rhs.floor;
        sv_ids = rhs.sv_ids;
        pois   = rhs.pois;
        return *this;
    }

    int type;

    int floor;

    std::vector<ID> sv_ids;

    std::vector<std::string> pois;
};

class EdgeInfo
{
public:
    EdgeInfo(double _length = 1, int _type = 0, double _width = 1) : width(_width), length(_length), type(_type) { }

    int type;

    double length;

    double width;
};

class Map : public DirectedGraph<NodeInfo, EdgeInfo>
{
public:
    /**
     * Check whether this map is empty or not
     * @return True if empty (true) or not (false)
     */
    bool isEmpty() const { return (countNodes() <= 0); }

    /**
     * Find a node using ID written in NodeInfo (time complexity: O(|N|))
     * @param id ID to search
     * @return A pointer to the found node (NULL if not exist)
     * @see getNode
     */
    Node* findNode(ID id) { return getNode(NodeInfo(id)); }

    /**
     * Find an edge using ID written in NodeInfo (time complexity: O(|N| + |E|))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @return A pointer to the found edge (NULL if not exist)
     * @see getEdge
     */
    Edge* findEdge(ID from, ID to) { return getEdge(NodeInfo(from), NodeInfo(to)); }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 A pointer to the first node
     * @param node2 A pointer to the second node
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(Node* node1, Node* node2)
    {
        if (node1 == NULL || node2 == NULL) return false;

        double dx = node1->data.x - node2->data.x;
        double dy = node1->data.y - node2->data.y;
        double dist = sqrt(dx * dx + dy * dy);
        Edge* edge1 = DirectedGraph<NodeInfo, EdgeInfo>::addEdge(node1, node2, EdgeInfo(dist));
        Edge* edge2 = DirectedGraph<NodeInfo, EdgeInfo>::addEdge(node2, node1, EdgeInfo(dist));
        return (edge1 != NULL) && (edge2 != NULL);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 Data of the first node
     * @param node2 Data of the second node
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(const NodeInfo& node1, const NodeInfo& node2)
    {
        Node* node1_ptr = getNode(node1);
        Node* node2_ptr = getNode(node2);
        return addRoad(node1_ptr, node2_ptr);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(ID node1, ID node2)
    {
        Node* node1_ptr = getNode(node1);
        Node* node2_ptr = getNode(node2);
        return addRoad(node1_ptr, node2_ptr);
    }
};

typedef GraphPainter<NodeInfo, EdgeInfo> MapPainter;

} // End of 'dg'

#endif // End of '__MAP__'
