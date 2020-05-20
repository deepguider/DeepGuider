#ifndef __SIMPLE_ROAD_MAP__
#define __SIMPLE_ROAD_MAP__

#include "core/basic_type.hpp"
#include "localizer/directed_graph.hpp"
#include "localizer/graph_painter.hpp"
#include <map>

namespace dg
{

/**
 * @brief Simple road map
 *
 * A <b>simple road map</b> is defined with its node as Point2ID and its edge with double-type cost.
 *
 * <b>File Format for RoadMap (CSV File)</b>
 *
 * A RoadMap file contains its data and connectivity in the form of texts.
 * In the text file, each line contains information for a node or an edge.
 *
 * In case of a node, a line starts from a prefix, <i>NODE</i>, and follows ID, X, and Y for a node.
 * The following example shows a node whose ID is 3 and located at (3.29, 10.18).
 * - NODE, 3, 3.29, 10.18
 *
 * In case of an edge, a line starts from a prefix, <i>EDGE</i>, and follows starting ID, destination ID, and its traversal cost.
 * The following example presents an edge that starts from a node (ID: 3) to other node (ID: 4).
 * Moreover, their traversal cost is assigned as 9.09.
 * - EDGE, 3, 4, 9.09
 * When a road map needs to include a bi-directional road, the road map contains a pair of edges as follows.
 * - EDGE, 3, 4, 9.09
 * - EDGE, 4, 3, 9.09
 */
class RoadMap : public DirectedGraph<Point2ID, double>
{
public:
    /**
     * The default constructor
     */
    RoadMap() { }

    /**
     * The copy constructor
     */
    RoadMap(const RoadMap& graph) { graph.copyTo(this); }

    /**
     * Read a map from the given file
     * @param filename The filename to read a map
     * @return Result of success (true) or failure (false)
     */
    bool load(const char* filename);

    /**
     * Write this map to the given file
     * @param filename The filename to write the map
     * @return Result of success (true) or failure (false)
     */
    bool save(const char* filename);

    /**
     * Check whether this map is empty or not
     * @return True if empty (true) or not (false)
     */
    bool isEmpty() const { return (countNodes() <= 0); }

    /**
     * Add a node (time complexity: O(1))
     * @param data Data to add
     * @return A pointer to the added node
     */
    Node* addNode(const Point2ID& data)
    {
        Node* ptr = DirectedGraph<Point2ID, double>::addNode(data);
        if (ptr != nullptr) m_node_lookup.insert(std::make_pair(data.id, ptr));
        return ptr;
    }

    /**
     * Find a node using its data (time complexity: O(1))
     * @param data Data to search
     * @return A pointer to the found node (nullptr if not exist)
     */
    Node* getNode(const Point2ID& data) { return getNode(data.id); }

    /**
     * Find a node using its ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (nullptr if not exist)
     */
    Node* getNode(ID id)
    {
        auto found = m_node_lookup.find(id);
        if (found == m_node_lookup.end()) return nullptr;
        return found->second;
    }

    /**
     * Add a directed edge between two nodes (time complexity: O(1))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @param cost Cost from the start to destination nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return A pointer to the added edge
     * @see addRoad for adding a bi-directional edge
     */
    Edge* addEdge(Node* from, Node* to, double cost = -1.0);

    /**
     * Add a directed edge between two nodes (time complexity: O(1))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @param cost Cost from the start to destination nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return A pointer to the added edge
     * @see addRoad for adding a bi-directional edge
     */
    Edge* addEdge(const Point2ID& from, const Point2ID& to, double cost = -1.0)
    {
        Node* from_ptr = getNode(from);
        Node* to_ptr = getNode(to);
        return addEdge(from_ptr, to_ptr, cost);
    }

    /**
     * Add a directed edge between two nodes (time complexity: O(1))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @param cost Cost from the start to destination nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return A pointer to the added edge
     * @see addRoad for adding a bi-directional edge
     */
    Edge* addEdge(ID from, ID to, double cost = -1.0)
    {
        Node* from_ptr = getNode(from);
        Node* to_ptr = getNode(to);
        return addEdge(from_ptr, to_ptr, cost);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 A pointer to the first node
     * @param node2 A pointer to the second node
     * @param cost Cost from the first to second nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(Node* node1, Node* node2, double cost = -1.0);

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 Data of the first node
     * @param node2 Data of the second node
     * @param cost Cost from the first to second nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(const Point2ID& node1, const Point2ID& node2, double cost = -1.0)
    {
        Node* node1_ptr = getNode(node1);
        Node* node2_ptr = getNode(node2);
        return addRoad(node1_ptr, node2_ptr, cost);
    }

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @param cost Cost from the first to second nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(ID node1, ID node2, double cost = -1.0)
    {
        Node* node1_ptr = getNode(node1);
        Node* node2_ptr = getNode(node2);
        return addRoad(node1_ptr, node2_ptr, cost);
    }

    /**
     * Find an edge using its connecting node data (time complexity: O(|E|))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @return A pointer to the found edge (nullptr if not exist)
     */
    Edge* getEdge(const Point2ID& from, const Point2ID& to) { return getEdge(from.id, to.id); }

    /**
     * Find an edge using its connecting node IDs (time complexity: O(|E|))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @return A pointer to the found edge (nullptr if not exist)
     * @see getEdge
     */
    Edge* getEdge(ID from, ID to);

    /**
     * Remove a node (time complexity: O(|V| |E|))<br>
     * This removes all edges connected from the node
     * @param node A node pointer to remove
     * @return True if successful (false if failed)
     */
    bool removeNode(Node* node)
    {
        if (node == nullptr) return false;
        m_node_lookup.erase(node->data.id);
        return DirectedGraph<Point2ID, double>::removeNode(node);
    }

    /**
     * Remove all nodes and edges
     * @return True if successful (false if failed)
     */
    bool removeAll()
    {
        m_node_lookup.clear();
        return DirectedGraph<Point2ID, double>::removeAll();
    }

    /**
     * Copy this to the other graph (time complexity: O(|N||E|))
     * @param dest A pointer to the other graph
     * @return True if successful (false if failed)
     */
    bool copyTo(RoadMap* dest) const;

    /**
     * Overriding the assignment operator
     * @param rhs A directed graph in the right-hand side
     * @return This object
     */
    RoadMap& operator=(const RoadMap& rhs)
    {
        rhs.copyTo(this);
        return *this;
    }

protected:
    /** A node lookup table whose key is 'ID' and value is the corresponding pointer to the node */
    std::map<ID, Node*> m_node_lookup;
};

/** A map visualizer for dg::RoadMap */
typedef GraphPainter<Point2ID, double> SimpleRoadPainter;

} // End of 'dg'

#endif // End of '__SIMPLE_ROAD_MAP__'
