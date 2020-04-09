#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"

namespace dg
{

class Edge;

/**
 * @brief Node information for the topological map
 */
class Node : public LatLon
{
public:
    /** Node type definition */
    enum
    {
        /** Basic node */
        NODE_BASIC = 0,

        /** Junction node (e.g. intersecting point, corner point, and end point of the road) */
        NODE_JUNCTION = 1,

        /** Door node (e.g. exit and entrance) */
        NODE_DOOR = 2,

        /** Elevator node */
        NODE_ELEVATOR = 3,

        /** Escalator node */
        NODE_ESCALATOR = 4,

        /** The number of node types */
        TYPE_NUM
    };

    /**
     * A constructor with member initialization
     * @param _id The given ID of this node
     * @param _lat The given latitude of this node (Unit: [deg])
     * @param _lon The given longitude of this node (Unit: [deg])
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID _id = 0, double _lat = 0, double _lon = 0, int _type = 0, int _floor = 0) : LatLon(_lat, _lon), id(_id), type(_type), floor(_floor) { }

    /**
     * A constructor with member initialization
     * @param _id The given ID of this node
     * @param ll The given latitude and longitude of this node (Unit: [deg])
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID _id, const LatLon& ll, int _type = 0, int _floor = 0) : LatLon(ll), id(_id), type(_type), floor(_floor) { }

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
        edge_ids = rhs.edge_ids;
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

    /** A vector of edgeIDs */
    std::vector<ID> edge_ids;
};

/**
 * @brief Edge information for the topological map
 */
class Edge
{
public:
    /** Edge type definition */
    enum
    {
        /** Sidewalk */
        EDGE_SIDEWALK = 0,

        /** General road (e.g. roads shared by pedestrians and cars, street, alley, corridor, ...) */
        EDGE_ROAD = 1,

        /** Crosswalk */
        EDGE_CROSSWALK = 2,

        /** Elevator section */
        EDGE_ELEVATOR = 3,

        /** Escalator section */
        EDGE_ESCALATOR = 4,

        /** Stair section */
        EDGE_STAIR = 5,

        /** The number of edge types */
        TYPE_NUM
    };

    /**
     * A constructor with member initialization
     * @param _id The given identifier
     * @param _length The given length of edge (Unit: [m])
     * @param _type The given type of edge
     * @param _directed The given flag whether the edge is undirected (false) or directed (true)
     * @param _node_id1 The given identifier of the first node
     * @param _node_id2 The given identifier of the second node
     */
    Edge(ID _id = 0, double _length = 1, int _type = 0, bool _directed = false, ID _node_id1 = 0, ID _node_id2 = 0) : id(_id), length(_length), type(_type), directed(_directed), node_id1(_node_id1), node_id2(_node_id2) { }

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
		node_id1 = rhs.node_id1;
		node_id2 = rhs.node_id2;
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

    /** A identifier of the first node */
    ID node_id1;

    /** A identifier of the second node */
    ID node_id2;
};

/**
 * @brief Point-of-interest (POI) information for the topological map
 */
class POI : public LatLon
{
public:
    /**
     * The default constructor
     */
    POI() { }

    /** The identifier */
    ID id;

    /** The name of this POI */
    std::wstring name;

    /** The floor of this POI */
    int floor;
};

/**
 * @brief Street-view information for the topological map
 */
class StreetView : public LatLon
{
public:
    /**
     * The default constructor
     */
    StreetView() { }

	/** The identifier */
	ID id;

    /** The floor of this StreetView */
    int floor;

    /** The date this StreetView was taken */
    std::string date;

    /** The given True north-based azimuths of this StreetView (Unit: [deg]) */
    double heading;
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
    int addNode(const Node& node)
    {
        nodes.push_back(node);
		int node_idx = nodes.size() - 1;
		lookup_nodes.insert(std::make_pair(node.id, node_idx));
		return node_idx;
    }

    /**
     * Add an edge between two nodes (time complexity: O(1))
     * @param node1 ID of the first node
     * @param node2 ID of the second node
     * @param info An edge information between two nodes<br>
     *  Edge::node1 and Edge::node2 are not necessary to be assigned.
     * @return A pointer to the added edge (`nullptr` if any node is not exist)
     */
    int addEdge(ID node1, ID node2, const Edge& info = Edge())
    {
        Node* node1_ptr = findNode(node1);
        Node* node2_ptr = findNode(node2);
        if (node1_ptr == nullptr || node2_ptr == nullptr) return -1;

        Edge edge = info;
        edge.node_id1 = node1_ptr->id;
        edge.node_id2 = node2_ptr->id;
        edges.push_back(edge);
        int edge_idx = edges.size()-1;
        node1_ptr->edge_ids.push_back(edges[edge_idx].id);
        if (!edge.directed) node2_ptr->edge_ids.push_back(edges[edge_idx].id);
        lookup_edges.insert(std::make_pair(edge.id, edge_idx));
        return edge_idx;
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
        return &nodes[found->second];
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
        return &edges[found->second];
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
        for (auto edge = from_ptr->edge_ids.begin(); edge != from_ptr->edge_ids.end(); edge++)
        {
			auto edge_ptr = findEdge(*edge);
            assert(edge_ptr->node_id1 != 0 && edge_ptr->node_id2 != 0); // Verify connection (comment this line if you want speed-up in DEBUG mode)
            if (edge_ptr->node_id1 == to || edge_ptr->node_id2 == to) return edge_ptr;
        }
        return nullptr;
    }

    /** A vector of nodes */
    std::vector<Node> nodes;

    /** A vector of edges */
    std::vector<Edge> edges;

    /** A vector of POIs */
    std::vector<POI> pois;

    /** A vector of Street-views */
    std::vector<StreetView> views;

protected:
    /** A hash table for finding nodes */
    std::map<ID, int> lookup_nodes;

    /** A hash table for finding nodes */
    std::map<ID, int> lookup_edges;
};

} // End of 'dg'

#endif // End of '__MAP__'
