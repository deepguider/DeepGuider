#ifndef __SIMPLE_ROAD_MAP__
#define __SIMPLE_ROAD_MAP__

#include "core/basic_type.hpp"
#include "core/directed_graph.hpp"
#include "core/graph_painter.hpp"

namespace dg
{

/**
 * @brief Simple road map
 *
 * A <b>simple road map</b> is defined with its node as Point2ID and its edge with double-type cost.
 *
 * <b>File Format for SimpleRoadMap (CSV File)</b>
 *
 * A SimpleRoadMap file contains its data and connectivity in the form of texts.
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
class SimpleRoadMap : public DirectedGraph<Point2ID, double>
{
public:
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
    bool isEmpty() const;

    /**
     * Find a node using ID written in Point2ID (time complexity: O(|N|))
     * @param id ID to search
     * @return A pointer to the found node (NULL if not exist)
     * @see getNode
     */
    Node* findNode(ID id) { return getNode(Point2ID(id)); }

    /**
     * Find an edge using ID written in Point2ID (time complexity: O(|N| + |E|))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @return A pointer to the found edge (NULL if not exist)
     * @see getEdge
     */
    Edge* findEdge(ID from, ID to) { return getEdge(Point2ID(from), Point2ID(to)); }

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
    bool addRoad(const Point2ID& node1, const Point2ID& node2, double cost = -1.0);

    /**
     * Add a bi-directional edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @param cost Cost from the first to second nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return True if successful (false if failed)
     * @see addEdge for adding a directional edge
     */
    bool addRoad(ID node1, ID node2, double cost = -1.0);

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
    Edge* addEdge(const Point2ID& from, const Point2ID& to, double cost = -1.0);

    /**
     * Add a directed edge between two nodes (time complexity: O(1))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @param cost Cost from the start to destination nodes (default: -1)<br>
     *  If the cost is given as a negative value, it is automatically assigned as Euclidean distance.
     * @return A pointer to the added edge
     * @see addRoad for adding a bi-directional edge
     */
    Edge* addEdge(ID from, ID to, double cost = -1.0);
};

/** A map visualizer for dg::SimpleRoadMap */
typedef GraphPainter<Point2ID, double> SimpleRoadPainter;

} // End of 'dg'

#endif // End of '__SIMPLE_ROAD_MAP__'
