#ifndef __DIRECTED_GRAPH__
#define __DIRECTED_GRAPH__

#include <list>

namespace dg
{

template<typename D, typename C> class NodeType;
template<typename D, typename C> class EdgeType;
template<typename D, typename C> class DirectedGraph;

/**
 * @brief Node for directed graphs
 *
 * A <b>node</b> is defined for graph data structure.
 * A node is connected to other nodes by edges defined as EdgeType.
 * It includes its contents in #data defined as the given template type, T.
 *
 * @see DirectedGraph A directed graph
 * @see EdgeType An edge in a graph
 */
template<typename D, typename C>
class NodeType
{
public:
    /**
     * The default constructor
     */
    NodeType() { }

    /**
     * A constructor with assigning #data
     * @param data Data
     */
    NodeType(D data) { this->data = data; }

    /**
     * Check equality with the other node
     * @param rhs A node in the right-hand side
     * @return True if both data are equal (false if not equal)
     */
    bool operator==(const NodeType<D, C>& rhs) const { return (data == rhs.data); }

    /**
     * Check inequality with the other node
     * @param rhs A node in the right-hand side
     * @return True if both data are not equal (false if equal)
     */
    bool operator!=(const NodeType<D, C>& rhs) const { return (data != rhs.data); }

    /** Data in this node */
    D data;

    friend class DirectedGraph<D, C>;

protected:
    /** A list for edges that start from this node */
    std::list<EdgeType<D, C> > m_edge_list;
};

/**
 * @brief Edge for directed graphs
 *
 * An <b>directed edge</b> is defined for graph data structure.
 * A edge connects from a node to the other which is described in #to.
 * It includes its cost in #cost defined as the given template type, C.
 *
 * @see DirectedGraph A directed graph
 * @see NodeType A node in a graph
 */
template<typename D, typename C>
class EdgeType
{
public:
    /**
     * The default constructor
     */
    EdgeType() { to = NULL; }

    /**
     * A constructor with assigning #to and #cost
     * @param to A pointer to the destination node
     * @param cost Cost to the destination node
     */
    EdgeType(NodeType<D, C>* to, const C& cost) { this->to = to; this->cost = cost; }

    /** A pointer to the destination node */
    NodeType<D, C>* to;

    /** Cost to the destination node */
    C cost;
};

/**
 * @brief Directed graph
 *
 * A <b>directed graph</b> is implemented with its auxiliary functions.
 * This simple implementation is intended for sparse directed graphs which has a small number of edges per a node.
 * It would be better to use a matrix to implement a dense graph.
 * Its edge is directed so that its connection is represented by the start node and destination node.
 * However, it can describe an undirected graph if every edge has its dual edge which connects from the destination node to the start node.
 *
 * @see Directed Graph (Wikipedia), http://en.wikipedia.org/wiki/Directed_graph
 */
template<typename D, typename C>
class DirectedGraph
{
public:
    /**
     * A node definition
     */
    typedef NodeType<D, C> Node;

    /**
     * An edge definition
     */
    typedef EdgeType<D, C> Edge;

    /**
     * A node iterator
     */
    typedef typename std::list< NodeType<D, C> >::iterator NodeItr;

    /**
     * An edge iterator
     */
    typedef typename std::list< EdgeType<D, C> >::iterator EdgeItr;

    /**
     * A constant node iterator
     */
    typedef typename std::list< NodeType<D, C> >::const_iterator NodeItrConst;

    /**
     * A constant edge iterator
     */
    typedef typename std::list< EdgeType<D, C> >::const_iterator EdgeItrConst;

    /**
     * The destructor
     */
    virtual ~DirectedGraph() { removeAll(); }

    /**
     * Add a node (time complexity: O(1))
     * @param data Data to add
     * @return A pointer to the added node
     */
    Node* addNode(const D& data) { m_node_list.push_back(data); return &(m_node_list.back()); }

    /**
     * Add an edge (time complexity: O(1))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @param cost Cost from the start to destination nodes
     * @return A pointer to the added edge
     */
    Edge* addEdge(const D& from, const D& to, const C& cost)
    {
        Node* from_ptr = getNode(from);
        Node* to_ptr = getNode(to);
        return addEdge(from_ptr, to_ptr, cost);
    }

    /**
     * Add an edge (time complexity: O(1))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @param cost Cost from the start to destination nodes
     * @return A pointer to the added edge
     */
    Edge* addEdge(Node* from, Node* to, const C& cost)
    {
        if ((from == NULL) || (to == NULL)) return NULL;
        from->m_edge_list.push_back(Edge(to, cost));
        return &(from->m_edge_list.back());
    }

    /**
     * Find a node using its data (time complexity: O(|N|))
     * @param data Data to search
     * @return A pointer to the found node (NULL if not exist)
     */
    Node* getNode(const D& data)
    {
        Node* node = NULL;
        for (NodeItr node_itr = getHeadNode(); node_itr != getTailNode(); node_itr++)
            if (node_itr->data == data) { node = &(*node_itr); break; }
        return node;
    }

    /**
     * Find an edge using its start and destination node (time complexity: O(|N| + |E|))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @return A pointer to the found edge (NULL if not exist)
     */
    Edge* getEdge(const D& from, const D& to)
    {
        Node* fnode = getNode(from);
        Node* tnode = getNode(to);
        if (fnode == NULL || tnode == NULL) return NULL;
        return getEdge(fnode, tnode);
    }

    /**
     * Find an edge using its start and destination node (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @return A pointer to the found edge (NULL if not exist)
     */
    Edge* getEdge(Node* from, Node* to)
    {
        if ((from == NULL) || (to == NULL)) return NULL;
        Edge* edge = NULL;
        for (EdgeItr edge_itr = getHeadEdge(from); edge_itr != getTailEdge(from); edge_itr++)
            if (edge_itr->to->data == to->data) { edge = &(*edge_itr); break; }
        return edge;
    }

    /**
     * Find an edge using its start and destination node (time complexity: O(|E|))
     * @param from An iterator of the start node
     * @param to An iterator of the destination node
     * @return A pointer to the found edge (NULL if not exist)
     */
    Edge* getEdge(NodeItr from, NodeItr to)
    {
        Edge* edge = NULL;
        for (EdgeItr edge_itr = getHeadEdge(from); edge_itr != getTailEdge(from); edge_itr++)
            if (edge_itr->to->data == to->data) { edge = &(*edge_itr); break; }
        return edge;
    }

    /**
     * Retrieve edge cost using its start and destination node (time complexity: O(|E|))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @return Cost of the found edge
     */
    C getEdgeCost(const D& from, const D& to)
    {
        Edge* edge = getEdge(from, to);
        if (edge == NULL) return C(-1);
        return edge->cost;
    }

    /**
     * Retrieve edge cost using its start and destination node (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @return Cost of the found edge
     */
    C getEdgeCost(Node* from, Node* to)
    {
        Edge* edge = getEdge(from, to);
        if (edge == NULL) return C(-1);
        return edge->cost;
    }

    /**
     * Retrieve edge cost using its start and destination node (time complexity: O(|E|))
     * @param from A const_iterator of the start node
     * @param to A const_iterator of the destination node
     * @return Cost of the found edge
     */
    C getEdgeCost(NodeItrConst from, NodeItrConst to) const
    {
        EdgeItrConst edge = getEdgeConst(from, to);
        if (edge == getTailEdgeConst(from)) return C(-1);
        return edge->cost;
    }

    /**
     * Check connectivity from a start node to a destination node (time complexity: O(|E|))
     * @param from Data of the start node
     * @param to Data of the destination node
     * @return True if they are connected (false if not connected)
     */
    bool isConnected(const D& from, const D& to) { return (getEdgeCost(from, to) >= 0); }

    /**
     * Check connectivity from a start node to a destination node (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @return True if they are connected (false if not connected)
     */
    bool isConnected(Node* from, Node* to) { return (getEdgeCost(from, to) >= 0); }

    /**
     * Check connectivity from a start node to a destination node (time complexity: O(|E|))
     * @param from A const_iterator of the start node
     * @param to A const_iterator of the destination node
     * @return True if they are connected (false if not connected)
     */
    bool isConnected(NodeItrConst from, NodeItrConst to) const { return (getEdgeCost(from, to) >= 0); }

    /**
     * Copy this to the other graph (time complexity: O(|N||E|))
     * @param dest A pointer to the other graph
     * @return True if successful (false if failed)
     */
    bool copyTo(DirectedGraph<D, C>* dest) const
    {
        if (dest == NULL) return false;

        dest->removeAll();
        for (NodeItrConst node = getHeadNodeConst(); node != getTailNodeConst(); node++)
            dest->addNode(node->data);
        for (NodeItrConst from = getHeadNodeConst(); from != getTailNodeConst(); from++)
            for (EdgeItrConst edge = getHeadEdgeConst(from); edge != getTailEdgeConst(from); edge++)
                dest->addEdge(dest->getNode(from->data), dest->getNode(edge->to->data), edge->cost);
        return true;
    }

    /**
     * Remove a node (time complexity: O(|V| |E|))<br>
     * This removes all edges connected from the node
     * @param node A node pointer to remove
     * @return True if successful (false if failed)
     */
    bool removeNode(Node* node)
    {
        if (node == NULL) return false;
        NodeItr is_found = getTailNode();
        for (NodeItr node_itr = getHeadNode(); node_itr != getTailNode(); node_itr++)
        {
            removeEdge(&(*node_itr), node);
            if (node_itr->data == node->data) is_found = node_itr;
        }
        if (is_found == m_node_list.end()) return false;
        m_node_list.erase(is_found);
        return true;
    }

    /**
     * Remove a node (time complexity: O(|V| |E|))<br>
     * This removes all edges connected from the node
     * @param node A node iterator of remove
     * @return True if successful (false if failed)
     */
    bool removeNode(NodeItr node)
    {
        NodeItr is_found = getTailNode();
        for (NodeItr node_itr = getHeadNode(); node_itr != getTailNode(); node_itr++)
        {
            removeEdge(node_itr, node);
            if (node_itr->data == node->data) is_found = node_itr;
        }
        if (is_found == m_node_list.end()) return false;
        m_node_list.erase(is_found);
        return true;
    }

    /**
     * Remove an edge (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param to A pointer to the destination node
     * @return True if successful (false if failed)
     */
    bool removeEdge(Node* from, Node* to)
    {
        if ((from == NULL) || (to == NULL)) return false;
        bool is_found = false;
        EdgeItr edge_itr = getHeadEdge(from);
        while (edge_itr != getTailEdge(from))
        {
            if (edge_itr->to->data == to->data)
            {
                edge_itr = from->m_edge_list.erase(edge_itr);
                is_found = true;
                continue;
            }
            edge_itr++;
        }
        return is_found;
    }

    /**
     * Remove an edge (time complexity: O(|E|))
     * @param from An iterator of the start node
     * @param to An iterator of the destination node
     * @return True if successful (false if failed)
     */
    bool removeEdge(NodeItr from, NodeItr to)
    {
        bool is_found = false;
        EdgeItr edge_itr = getHeadEdge(from);
        while (edge_itr != getTailEdge(from))
        {
            if (edge_itr->to->data == to->data)
            {
                edge_itr = from->m_edge_list.erase(edge_itr);
                is_found = true;
                continue;
            }
            edge_itr++;
        }
        return is_found;
    }

    /**
     * Remove all nodes and edges
     * @return True if successful (false if failed)
     */
    bool removeAll() { m_node_list.clear(); return true; }

    /**
     * Count the number of all nodes (time complexity: O(1))
     * @return The number of nodes
     */
    int countNodes() const { return m_node_list.size(); }

    /**
     * Count the number of edges starting from the given node (time complexity: O(1))
     * @param node Data of the node
     * @return The number of edges
     */
    int countEdges(const D& data) const
    {
        NodeItrConst node_itr = getNodeConst(data);
        if (node_itr == getTailNodeConst()) return 0;
        return node_itr->m_edge_list.size();
    }

    /**
     * Count the number of edges starting from the given node (time complexity: O(1))
     * @param node A pointer to the node
     * @return The number of edges
     */
    int countEdges(const Node* node) const
    {
        if (node == NULL) return 0;
        return node->m_edge_list.size();
    }

    /**
     * Count the number of edges starting from the given node (time complexity: O(1))
     * @param node An iterator of the node
     * @return The number of edges
     */
    int countEdges(NodeItrConst node) const { return node->m_edge_list.size(); }

    /**
     * Get an iterator of the first node in this graph (time complexity: O(1))
     * @return An iterator of the first node
     * @see getTailNode
     */
    NodeItr getHeadNode() { return m_node_list.begin(); }

    /**
     * Get an iterator of the ending node in this graph (time complexity: O(1))
     * @return An iterator of the ending node
     * @see getHeadNode
     */
    NodeItr getTailNode() { return m_node_list.end(); }

    /**
     * Get an iterator of the first edge from the given node (time complexity: O(1))
     * @param node A pointer to a node
     * @return An iterator of the first edge
     * @see getTailEdge
     */
    EdgeItr getHeadEdge(Node* node) { return node->m_edge_list.begin(); }

    /**
     * Get an iterator of the first edge from the given node (time complexity: O(1))
     * @param node An iterator of a node
     * @return An iterator of the first edge
     * @see getTailEdge
     */
    EdgeItr getHeadEdge(NodeItr node) { return node->m_edge_list.begin(); }

    /**
     * Get an iterator of the ending edge from the given node (time complexity: O(1))
     * @param node A pointer to a node
     * @return An iterator of the ending edge
     * @see getHeadEdge
     */
    EdgeItr getTailEdge(Node* node) { return node->m_edge_list.end(); }

    /**
     * Get an iterator of the ending edge from the given node (time complexity: O(1))
     * @param node An iterator of a node
     * @return An iterator of the ending edge
     * @see getHeadEdge
     */
    EdgeItr getTailEdge(NodeItr node) { return node->m_edge_list.end(); }

    /**
     * Find a node using its data (time complexity: O(|N|))
     * @param data Data to search
     * @return A const_iterator of the found node (getTailNodeConst() if not exist)
     */
    NodeItrConst getNodeConst(const D& data) const
    {
        NodeItrConst node_itr = getHeadNodeConst();
        for (; node_itr != getTailNodeConst(); node_itr++)
            if (node_itr->data == data) break;
        return node_itr;
    }

    /**
     * Find an edge using its start and destination node (time complexity: O(|E|))
     * @param from A const_iterator of the start node
     * @param to A const_iterator of the destination node
     * @return A const_iterator of the found edge (getTailEdgeConst(from) if not exist)
     */
    EdgeItrConst getEdgeConst(NodeItrConst from, NodeItrConst to) const
    {
        EdgeItrConst edge_itr = getHeadEdgeConst(from);
        for (; edge_itr != getTailEdgeConst(from); edge_itr++)
            if (edge_itr->to->data == to->data) break;
        return edge_itr;
    }

    /**
     * Get a const_iterator of the first node in this graph (time complexity: O(1))
     * @return A const_iterator of the first node
     * @see getTailNodeConst
     */
    NodeItrConst getHeadNodeConst() const { return m_node_list.cbegin(); }

    /**
     * Get a const_iterator of the ending node in this graph (time complexity: O(1))
     * @return A const_iterator of the ending node
     * @see getHeadNodeConst
     */
    NodeItrConst getTailNodeConst() const { return m_node_list.cend(); }

    /**
     * Get a const_iterator of the first edge from the given node (time complexity: O(1))
     * @param node A constant pointer to a node
     * @return A const_iterator of the first edge
     * @see getTailEdgeConst
     */
    EdgeItrConst getHeadEdgeConst(const Node* node) const { return node->m_edge_list.cbegin(); }

    /**
     * Get a const_iterator of the first edge from the given node (time complexity: O(1))
     * @param node A const_iterator of a node
     * @return A const_iterator of the first edge
     * @see getTailEdgeConst
     */
    EdgeItrConst getHeadEdgeConst(NodeItrConst node) const { return node->m_edge_list.cbegin(); }

    /**
     * Get a const_iterator of the ending edge from the given node (time complexity: O(1))
     * @param node A constant pointer to a node
     * @return A const_iterator of the ending edge
     * @see getHeadEdgeConst
     */
    EdgeItrConst getTailEdgeConst(const Node* node) const { return node->m_edge_list.cend(); }

    /**
     * Get a const_iterator of the ending edge from the given node (time complexity: O(1))
     * @param node A const_iterator of a node
     * @return A const_iterator of the ending edge
     * @see getHeadEdgeConst
     */
    EdgeItrConst getTailEdgeConst(NodeItrConst node) const { return node->m_edge_list.cend(); }

protected:
    /** A list for all edges in this graph */
    std::list<Node> m_node_list;

}; // End of 'DirectedGraph'

} // End of 'dg'

#endif // End of '__DIRECTED_GRAPH__'
