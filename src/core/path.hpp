#ifndef __PATH__
#define __PATH__

#include "core/basic_type.hpp"
#include "utils/utm_converter.hpp"

namespace dg
{

/**
 * @brief Path element
 */
struct PathNode : public Point2
{
    /**
     * Node ID<br>
     * ID = 0 for temporal nodes (e.g. starting & destination node)
     */
    ID node_id;

    /**
     * Edge ID<br>
     * ID = 0 for temporal edges (e.g. starting & destination edge)
     */
    ID edge_id;

    /**
     * A constructor with member initialization
     * @param x The given x coordinate of path node (Unit: [m])
     * @param y The given y coordinate of path node (Unit: [m])
     * @param _node_id ID of this path node
     * @param _edge_id ID of this path edge
     */
    PathNode(double x = 0, double y = 0, ID _node_id = 0, ID _edge_id = 0) : Point2(x, y), node_id(_node_id), edge_id(_edge_id) { }

    /**
     * A constructor with member initialization
     * @param p The given 2D coordinate of path node
     * @param _node_id ID of this path node
     * @param _edge_id ID of this path edge
     */
    PathNode(const Point2& p, ID _node_id = 0, ID _edge_id = 0) : Point2(p), node_id(_node_id), edge_id(_edge_id) { }

    /**
     * A constructor with member initialization
     * @param pid The given node point with ID
     * @param _edge_id ID of this path edge
     */
    PathNode(const Point2ID& pid, ID _edge_id = 0) : Point2(pid), node_id(pid.id), edge_id(_edge_id) { }
};

/**
 * @brief Path definition
 *
 * A path is represented by a sequence of node points and edges defined in PathNode.
 */
class Path
{
public:
    /**
     * The default constructor
     */
    Path() { }

    /** A series of nodes and edges consisting of a path */
    std::vector<PathNode> pts;

    /** Clear path */
    void clear() { pts.clear(); }

    /** Check if it is empty */
    bool empty() const { return pts.empty(); }
};

} // End of 'dg'

#endif // End of '__PATH__'
