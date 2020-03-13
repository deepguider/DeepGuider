#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"

namespace dg
{

/** Node type definition of the topological map */
enum
{
    /** Basic node */
    NT_BS = 0,

    /** Junction node (e.g. intersecting point, corner point, and end point of the road) */
    NT_JT = 1,

    /** Door node (e.g. exit and entrance) */
    NT_DR = 2,

    /** Elevator node */
    NT_EV = 3,

    /** Escalator node */
    NT_ES = 4,

    /** The number of node types */
    NT_NUM
};

/** Edge type definition of the topological map */
enum
{
    /** Sidewalk */
    ET_SD = 0,

    /** Middle road (e.g. lane, ginnel, roads shared by pedestrians and cars, ...) */
    ET_MD = 1,

    /** Crosswalk */
    ET_CR = 2,

    /** Doorway */
    ET_DR = 3,

    /** Elevator section */
    ET_EV = 4,

    /** Escalator section */
    ET_ES = 5,

    /** The number of edge types */
    ET_NUM
};

class Edge;

/**
 * @brief Node information for the topological map
 */
class Node : public LatLon
{
public:
    /**
     * A constructor with member initialization
     * @param _id The given ID of this node
     * @param _lat The given latitude of this node (Unit: [deg])
     * @param _lon The given longitude of this node (Unit: [deg])
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID _id = 0, double _lat = 0, double _lon = 0, int _type = 0, int _floor = 0) : id(_id), LatLon(_lat, _lon), type(_type), floor(_floor) { }

    /**
     * A constructor with member initialization
     * @param _id The given ID of this node
     * @param ll The given latitude and longitude of this node (Unit: [deg])
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID _id, const LatLon& ll, int _type = 0, int _floor = 0) : id(_id), LatLon(ll), type(_type), floor(_floor) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Node& operator=(const Node& rhs)
    {
        id     = rhs.id;
        lat    = rhs.lat;
        lon    = rhs.lon;
        type   = rhs.type;
        floor  = rhs.floor;
        edge_list = rhs.edge_list;
        return *this;
    }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const Node& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const Node& rhs) const { return (id != rhs.id); }

    /** The identifier */
    ID id;

    /**
     * The type of node
     * @see NT_BS, NT_JT, NT_DR, NT_EV, NT_ES
     */
    int type;

    /**
     * The floor of node<br>
     * The ground (outdoor) is given as 0, the default value.
     */
    int floor;

    /** A list of edges */
    std::vector<Edge*> edge_list;
};

/**
 * @brief Edge information for the topological map
 */
class Edge
{
public:
    /**
     * A constructor with member initialization
     * @param _id The given identifier
     * @param _length The given length of edge (Unit: [m])
     * @param _type The given type of edge
     * @param _directed The given flag whether the edge is undirected (false) or directed (true)
     * @param _node1 The given pointer to the first node
     * @param _node2 The given pointer to the second node
     */
    Edge(ID _id = 0, double _length = 1, int _type = 0, bool _directed = false, Node* _node1 = nullptr, Node* _node2 = nullptr) : id(_id), length(_length), type(_type), directed(_directed), node1(_node1), node2(_node2) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Edge& operator=(const Edge& rhs)
    {
        id     = rhs.id;
        length = rhs.length;
        type   = rhs.type;
        node1  = rhs.node1;
        node2  = rhs.node2;
        return *this;
    }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const Edge& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const Edge& rhs) const { return (id != rhs.id); }

    /** The identifier */
    ID id;

    /** The length of edge (Unit: [m]) */
    double length;

    /**
     * The type of edge
     * @see ET_SD, ET_MD, ET_CR, ET_DR, ET_EV, ET_ES
     */
    int type;

    /** A flag whether this edge is undirected (false) or directed (true) */
    bool directed;

    /** A pointer to the first node */
    Node* node1;

    /** A pointer to the second node */
    Node* node2;
};

/**
 * @brief Point-of-interest (POI) information for the topological map
 */
class POI
{
public:
	/**
	 * The default constructor
	 */
	POI() { }

	/** The identifier */
	ID id;

	/** The name of this POI */
	std::string name;

	/** The floor of this POI */
	int floor;

	/** The given latitude of this POI (Unit: [deg]) */
	double lat;

	/** The given longitude of this POI (Unit: [deg]) */
	double lon;
};

/**
 * @brief Street-view information for the topological map
 */
class StreetView
{
public:
	/**
	 * The default constructor
	 */
	StreetView() { }

	/** The identifier */
	std::string id;

	/** The floor of this StreetView */
	int floor;

	/** The date this StreetView was taken */
	std::string date;

	/** The given True north-based azimuths of this StreetView (Unit: [deg]) */
	double heading;

	/** The given latitude of this POI (Unit: [deg]) */
	double lat;

	/** The given longitude of this POI (Unit: [deg]) */
	double lon;
};

/**
 * @brief A topological map
 */
class Map
{
public:
    /**
     * Add a node (time complexity: O(1))
     * @param data Data to add
     * @return A pointer to the added node
     */
    Node* addNode(const Node& node)
    {
        nodes.push_back(node);
        Node* node_ptr = &(nodes.back());
        lookup_nodes.insert(std::make_pair(node.id, node_ptr));
        return node_ptr;
    }

    /**
     * Add an edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @param info An edge information between two nodes<br>
     *  Edge::node1 and Edge::node2 are not necessary to be assigned.
     * @return A pointer to the added edge (`nullptr` if any node is not exist)
     */
    Edge* addEdge(ID node1, ID node2, const Edge& info = Edge())
    {
        Node* node1_ptr = findNode(node1);
        Node* node2_ptr = findNode(node2);
        if (node1_ptr == nullptr || node2_ptr == nullptr) return nullptr;

        Edge edge = info;
        edge.node1 = node1_ptr;
        edge.node2 = node2_ptr;
        edges.push_back(edge);
        Edge* edge_ptr = &(edges.back());
        node1_ptr->edge_list.push_back(edge_ptr);
        if (!edge.directed) node2_ptr->edge_list.push_back(edge_ptr);
        lookup_edges.insert(std::make_pair(edge.id, edge_ptr));
        return edge_ptr;
    }

    /**
     * Find a node using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    Node* findNode(ID id)
    {
        assert(nodes.size() == lookup_nodes.size() && lookup_nodes.count(id) <= 1); // Verify ID uniqueness (comment this line if you want speed-up in DEBUG mode)
        auto found = lookup_nodes.find(id);
        if (found == lookup_nodes.end()) return nullptr;
        return found->second;
    }

    /**
     * Find an edge using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found edge (`nullptr` if not exist)
     */
    Edge* findEdge(ID id)
    {
        assert(edges.size() == lookup_edges.size() && lookup_edges.count(id) <= 1); // Verify ID uniqueness (comment this line if you want speed-up in DEBUG mode)
        auto found = lookup_edges.find(id);
        if (found == lookup_edges.end()) return nullptr;
        return found->second;
    }

    /**
     * Find an edge using ID (time complexity: O(|E|))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @return A pointer to the found edge (`nullptr` if not exist)
     */
    Edge* findEdge(ID from, ID to)
    {
        Node* from_ptr = findNode(from);
        if (from_ptr == nullptr) return nullptr;
        for (auto edge = from_ptr->edge_list.begin(); edge != from_ptr->edge_list.end(); edge++)
        {
            assert((*edge)->node1 != nullptr && (*edge)->node2 != nullptr); // Verify connection (comment this line if you want speed-up in DEBUG mode)
            if ((*edge)->node1->id == to || (*edge)->node2->id == to) return *edge;
        }
        return nullptr;
    }

    /** A list of nodes */
    std::list<Node> nodes;

    /** A list of edges */
    std::list<Edge> edges;

    /** A list of POIs */
    std::list<POI> pois;

    /** A list of Street-views */
    std::list<StreetView> views;

protected:
    /** A hash table for finding nodes */
    std::map<ID, Node*> lookup_nodes;

    /** A hash table for finding nodes */
    std::map<ID, Edge*> lookup_edges;
};

} // End of 'dg'

#endif // End of '__MAP__'
