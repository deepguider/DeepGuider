#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"
#include "core/directed_graph.hpp"
//#include "core/graph_painter.hpp"

namespace dg
{

 /**
  * @brief A node information for the topological map
  *
  * TODO
  */
class NodeInfo : public LatLon
{
public:
    /**
     * The default constructor
     * TODO
     */
    NodeInfo(ID _id = 0, double _lon = 0, double _lat = 0, int _type = 0, int _floor = 0) : id(_id), LatLon(_lon, _lat), type(_type), floor(_floor) { }

    /**
     * A constructor with initialization
     * TODO
     */
    NodeInfo(ID _id, const LatLon& ll, int _type = 0, int _floor = 0) : id(_id), LatLon(ll), type(_type), floor(_floor) { }

    /**
     * Overriding the assignment operator
     * @param rhs NodeInfo in the right-hand side
     * @return The assigned instance
     */
    NodeInfo& operator=(const NodeInfo& rhs)
    {
        id     = rhs.id;
        lat    = rhs.lat;
        lon    = rhs.lon;
        type   = rhs.type;
        floor  = rhs.floor;
        sv_ids = rhs.sv_ids;
        pois   = rhs.pois;
        return *this;
    }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const NodeInfo& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const NodeInfo& rhs) const { return (id != rhs.id); }

    /** The identifier */
    ID id;

    /** The type of node */
    int type;

    /**
     * The floor of node<br>
     * The ground (outdoor) is given as 0, the default value.
     */
    int floor;

    /** An ID set of connected street-view images */
    std::vector<ID> sv_ids;

    /** A text set of connected POIs */
    std::vector<std::string> pois;
};

 /**
  * @brief An edge information for the topological map
  *
  * TODO
  */
class EdgeInfo
{
public:
    /**
     * The default constructor
     * TODO
     */
    EdgeInfo(double _length = 1, double _width = 1, int _type = 0) : length(_length), width(_width), type(_type) { }

    /** The length of edge (Unit: [m]) */
    double length;

    /** The width of edge (Unit: [m]) */
    double width;

    /** The type of edge */
    int type;
};

 /**
  * @brief A topological map
  *
  * TODO
  */
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
     * @param info An edge information between two nodes
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(Node* node1, Node* node2, const EdgeInfo& info = EdgeInfo())
    {
        if (node1 == NULL || node2 == NULL) return false;

        Edge* edge1 = DirectedGraph<NodeInfo, EdgeInfo>::addEdge(node1, node2, info);
        Edge* edge2 = DirectedGraph<NodeInfo, EdgeInfo>::addEdge(node2, node1, info);
        return (edge1 != NULL) && (edge2 != NULL);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 Data of the first node
     * @param node2 Data of the second node
     * @param info An edge information between two nodes
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(const NodeInfo& node1, const NodeInfo& node2, const EdgeInfo& info = EdgeInfo())
    {
        Node* node1_ptr = getNode(node1);
        Node* node2_ptr = getNode(node2);
        return addRoad(node1_ptr, node2_ptr, info);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @param info An edge information between two nodes
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(ID node1, ID node2, const EdgeInfo& info = EdgeInfo())
    {
        Node* node1_ptr = findNode(node1);
        Node* node2_ptr = findNode(node2);
        return addRoad(node1_ptr, node2_ptr, info);
    }
};

} // End of 'dg'

#endif // End of '__MAP__'
