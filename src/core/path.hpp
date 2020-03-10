#ifndef __PATH__
#define __PATH__

#include "map.hpp"

namespace dg
{

/**
 * @brief Path element for pointing its node and edge
 */
struct PathElement
{
    /**
     * A constructor with member initialization
     * @param _node A pointer to a node
     * @param _edge A pointer to an edge
     */
    PathElement(Node* _node = nullptr, Edge* _edge = nullptr) : node(_node), edge(_edge) { }

    /** A pointer to a node */
    Node* node;

    /**
     * A pointer to an edge<br>
     * The last element has 'nullptr'.
     */
    Edge* edge;
};

/**
 * @brief Path definition
 *
 * A path is represented by a sequence of points defined in PathElement.
 */
class Path
{
public:
    /**
     * The default constructor
     */
    Path() { }

    /** A series of nodes and edges for a path */
    std::vector<PathElement> pts;
};

} // End of 'dg'

#endif // End of '__PATH__'
