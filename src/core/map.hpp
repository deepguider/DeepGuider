#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"
#include "path.hpp"
#include "utils/utm_converter.hpp"

namespace dg
{

struct Edge;

/**
 * @brief Node information for the topological map
 */
struct Node : public Point2ID
{
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
     * The type of node
     * @see NT_BS, NT_JT, NT_DR, NT_EV, NT_ES
     */
    int type;

    /**
     * The floor of node<br>
     * The ground (outdoor) is given as 0, the default value.
     */
    int floor;

    /** A vector of edge IDs */
    std::vector<ID> edge_ids;

    /**
     * A constructor with member initialization
     * @param id The given ID of this node
     * @param x The given x coordinate of this node (Unit: [m])
     * @param y The given y coordinate of this node (Unit: [m])
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID id = 0, double x = 0, double y = 0, int _type = 0, int _floor = 0) : Point2ID(id, x, y), type(_type), floor(_floor) { }

    /**
     * A constructor with member initialization
     * @param id The given ID of this node
     * @param p The given 2D point
     * @param _type The given type of this node
     * @param _floor The given floor of this node
     */
    Node(ID id, const Point2& p, int _type = 0, int _floor = 0) : Point2ID(id, p), type(_type), floor(_floor) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Node& operator=(const Node& rhs)
    {
        id     = rhs.id;
        x      = rhs.x;
        y      = rhs.y;
        type   = rhs.type;
        floor  = rhs.floor;
        edge_ids = rhs.edge_ids;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Node& operator=(const Point2ID& rhs)
    {
        id = rhs.id;
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Node& operator=(const Point2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
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
};

/**
 * @brief Edge information for the topological map
 */
struct Edge
{
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

        /** Doorway section */
        EDGE_DOORWAY = 6,

        /** The number of edge types */
        TYPE_NUM
    };

    /** The identifier */
    ID id;

    /**
     * The type of edge
     * @see ET_SD, ET_MD, ET_CR, ET_DR, ET_EV, ET_ES
     */
    int type;

    /** A identifier of the first node */
    ID node_id1;

    /** A identifier of the second node */
    ID node_id2;

    /** The length of edge (Unit: [m]) */
    double length;

    /** A flag whether this edge is undirected (false) or directed (true) */
    bool directed;

    /**
     * Side of the road where the edge (node_id1 -> node_id2) is located
     * LR_LEFT: left-side (edge from node_id1 to node_id2 is located left side of a road)
     * LR_RIGHT: right-size (edge from node_id1 to node_id2 is located right side of a road)
     * LR_NONE: side info is not applicable nor undefined for this edge
     */
    int lr_side;
    enum { LR_LEFT = 0, LR_RIGHT = 2, LR_NONE = -1 };

    /**
     * A constructor with member initialization
     * @param _id The given identifier
     * @param _type The given type of edge
     * @param _node_id1 The given identifier of the first node
     * @param _node_id2 The given identifier of the second node
     * @param _length The given length of edge (Unit: [m])
     * @param _directed The given flag whether the edge is undirected (false) or directed (true)
     */
    Edge(ID _id = 0, int _type = 0, ID _node_id1 = 0, ID _node_id2 = 0, double _length = -1, bool _directed = false, int _lr_side = LR_NONE) : id(_id), type(_type), node_id1(_node_id1), node_id2(_node_id2), length(_length), directed(_directed), lr_side(_lr_side) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    Edge& operator=(const Edge& rhs)
    {
        id     = rhs.id;
        type = rhs.type;
		node_id1 = rhs.node_id1;
		node_id2 = rhs.node_id2;
        length = rhs.length;
        directed = rhs.directed;
        lr_side = rhs.lr_side;
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
};

/**
 * @brief Point-of-interest (POI) information for the topological map
 */
struct POI : public Point2ID
{
    /** The name of this POI */
    std::wstring name;

    /**
    * The floor of this POI<br>
    * The first floor (indoor) is given as 1, basement floors are given as -1, -2, ...<br>
    */
    int floor;

    /**
     * A constructor with member initialization
     * @param id The given ID of this POI
     * @param x The given x coordinate of this POI (Unit: [m])
     * @param y The given y coordinate of this POI (Unit: [m])
     * @param _name The given name of this POI
     * @param _floor The given floor of this POI
     */
    POI(ID id = 0, double x = 0, double y = 0, std::wstring _name = L"", int _floor = 1) : Point2ID(id, x, y), name(_name), floor(_floor) { }

    /**
     * A constructor with member initialization
     * @param id The given ID of this POI
     * @param p The given 2D point of this POI
     * @param _name The given name of this POI
     * @param _floor The given floor of this POI
     */
    POI(ID id, const Point2& p, std::wstring _name, int _floor = 1) : Point2ID(id, p), name(_name), floor(_floor) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    POI& operator=(const POI& rhs)
    {
        id = rhs.id;
        x = rhs.x;
        y = rhs.y;
        name = rhs.name;
        floor = rhs.floor;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    POI& operator=(const Point2ID& rhs)
    {
        id = rhs.id;
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    POI& operator=(const Point2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const POI& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const POI& rhs) const { return (id != rhs.id); }
};

/**
 * @brief Street-view information for the topological map
 */
struct StreetView : public Point2ID
{
    /**
    * The floor of this StreetView<br>
    * The ground(outdoor) streetviews are given as 0, the default value.<br>
    * The indoor streetviews are given as 1, 2, ... for the first floor, second floor, ... and as -1, -2, ... for basement floors.
    */
    int floor;

    /** The given True north-based azimuths of this StreetView (Unit: [deg]) */
    double heading;

    /** The date this StreetView was taken (Format: [yyyy-mm-dd]) */
    std::string date;

    /**
     * A constructor with member initialization
     * @param id The given ID of this StreetView
     * @param x The given x coordinate of the camera that took this StreetView (Unit: [m])
     * @param y The given y coordinate of the camera that took this StreetView (Unit: [m])
     * @param _floor The given floor of this StreetView
     * @param _heading The given True north-based azimuths of this StreetView (Unit: [rad])
     * @param _date The given date this StreetView was taken (Format: [yyyy-mm-dd])
     */
    StreetView(ID id = 0, double x = 0, double y = 0, int _floor = 0, double _heading = 0, std::string _date = "") : Point2ID(id, x, y), floor(_floor), heading(_heading), date(_date) { }

    /**
     * A constructor with member initialization
     * @param id The given ID of this StreetView
     * @param p The given 2D point of the camera that took this StreetView (Unit: [m])
     * @param _floor The given floor of this StreetView
     * @param _heading The given True north-based azimuths of this StreetView (Unit: [rad])
     * @param _date The given date this StreetView was taken (Format: [yyyy-mm-dd])
     */
    StreetView(ID id, const Point2& p, int _floor = 0, double _heading = 0, std::string _date = "") : Point2ID(id, p), floor(_floor), heading(_heading), date(_date) { }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    StreetView& operator=(const StreetView& rhs)
    {
        id = rhs.id;
        x = rhs.x;
        y = rhs.y;
        floor = rhs.floor;
        heading = rhs.heading;
        date = rhs.date;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    StreetView& operator=(const Point2ID& rhs)
    {
        id = rhs.id;
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /**
     * Overriding the assignment operator
     * @param rhs The right-hand side
     * @return The assigned instance
     */
    StreetView& operator=(const Point2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const StreetView& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const StreetView& rhs) const { return (id != rhs.id); }
};

/**
 * A ID iterator
 */
typedef typename std::vector<ID>::iterator IDItr;

/**
 * A node iterator
 */
typedef typename std::vector<Node>::iterator NodeItr;

/**
 * An edge iterator
 */
typedef typename std::vector<Edge>::iterator EdgeItr;

/**
 * A POI iterator
 */
typedef typename std::vector<POI>::iterator POIItr;

/**
 * An streetview iterator
 */
typedef typename std::vector<StreetView>::iterator StreetViewItr;

/**
 * A const ID iterator
 */
typedef typename std::vector<ID>::const_iterator IDItrConst;

/**
 * A const node iterator
 */
typedef typename std::vector<Node>::const_iterator NodeItrConst;

/**
 * An const edge iterator
 */
typedef typename std::vector<Edge>::const_iterator EdgeItrConst;

/**
 * A const POI iterator
 */
typedef typename std::vector<POI>::const_iterator POIItrConst;

/**
 * An const streetview iterator
 */
typedef typename std::vector<StreetView>::const_iterator StreetViewItrConst;

/** Calculate shortest distance from a point to a line segment */
std::pair<double, Point2> calcDist2FromLineSeg(const Point2& from, const Point2& to, const Pose2& p, double turn_weight = 0);


/**
 * @brief A topological map
 */
class Map : public UTMConverter
{
public:

    /**
     * The default constructor
     */
    Map() { }

    /**
     * The copy constructor
     */
    Map(const Map& map) { map.copyTo(this); }

    /**
     * Read a map from the given file
     * @param filename The filename to read a map
     * @param load_from_latlon If true, node position is assumed to be saved in geodesic (default), in UTM otherwise.
     * @return Result of success (true) or failure (false)
     */
    bool load(const char* filename, bool load_from_latlon = true);

    /**
     * Write this map to the given file
     * @param filename The filename to write the map
     * @param save_as_latlon If true, save node position in geodesic (default), in UTM otherwise.
     * @return Result of success (true) or failure (false)
     */
    bool save(const char* filename, bool save_as_latlon = true) const;

    /**
     * Add a node (time complexity: O(1))
     * @param node Node to add
     * @return Result of success (true) or failure (false)
     */
    bool addNode(const Node& node)
    {
        size_t node_idx = nodes.size();
        auto result = lookup_nodes.insert(std::make_pair(node.id, node_idx));
        if (!result.second) return false;
        nodes.push_back(node);
        m_router_valid = false;
        m_map_rect_valid = false;
        return true;
    }

    /**
     * Delete a node (time complexity: O(1))
     * @param id Node ID to delete
     */
    void deleteNode(ID id)
    {
        auto found = lookup_nodes.find(id);
        if (found == lookup_nodes.end()) return;
        Node n = nodes[found->second];
        for (auto it = n.edge_ids.begin(); it != n.edge_ids.end(); it++)
        {
            deleteEdge(*it);
        }
        nodes.erase(nodes.begin() + found->second);

        lookup_nodes.clear();
        for (auto idx = 0; idx < nodes.size(); idx++)
        {
            lookup_nodes.insert(std::make_pair(nodes[idx].id, idx));
        }
        m_router_valid = false;
        m_map_rect_valid = false;
    }

    /**
     * Add an edge (time complexity: O(1))
     * @param edge Edge to add
     * @param auto_length Length of edge is set to be Euclidean distance between two nodes
     * @return Result of success (true) or failure (false)
     */
    bool addEdge(const Edge& _edge, bool auto_length = false)
    {
        Edge edge = _edge;
        Node* node1_ptr = getNode(edge.node_id1);
        Node* node2_ptr = getNode(edge.node_id2);
        if (node1_ptr == nullptr || node2_ptr == nullptr) return false;

        size_t edge_idx = edges.size();
        auto result = lookup_edges.insert(std::make_pair(edge.id, edge_idx));
        if (!result.second) return false;

        if (auto_length || edge.length < 0)
        {
            double dx = node1_ptr->x - node2_ptr->x;
            double dy = node1_ptr->y - node2_ptr->y;
            edge.length = sqrt(dx * dx + dy * dy);
        }
        edges.push_back(edge);
        node1_ptr->edge_ids.push_back(edge.id);
        if (!edge.directed) node2_ptr->edge_ids.push_back(edge.id);
        m_router_valid = false;
        return true;
    }

    /**
     * Add an edge (time complexity: O(1))
     * @param node_from First Node ID to connect
     * @param node_to Second Node ID to connec
     * @param edge_data Edge data (node ids in edge_data is ignored)
     * @param auto_length Length of edge is set to be Euclidean distance between two nodes
     * @return Result of success (true) or failure (false)
     */
    bool addEdge(ID node_from, ID node_to, const Edge& edge_data, bool auto_length = false)
    {
        Node* node1_ptr = getNode(node_from);
        Node* node2_ptr = getNode(node_to);
        if (node1_ptr == nullptr || node2_ptr == nullptr) return false;

        Edge edge = edge_data;
        edge.node_id1 = node_from;
        edge.node_id2 = node_to;
        size_t edge_idx = edges.size();
        auto result = lookup_edges.insert(std::make_pair(edge.id, edge_idx));
        if (!result.second) return false;

        if (auto_length || edge.length < 0)
        {
            double dx = node1_ptr->x - node2_ptr->x;
            double dy = node1_ptr->y - node2_ptr->y;
            edge.length = sqrt(dx * dx + dy * dy);
        }
        edges.push_back(edge);
        node1_ptr->edge_ids.push_back(edge.id);
        if (!edge.directed) node2_ptr->edge_ids.push_back(edge.id);
        m_router_valid = false;
        return true;
    }

    /**
     * Delete an edge (time complexity: O(1))
     * @param id Edge ID to delete
     */
    void deleteEdge(ID id)
    {
        auto found = lookup_edges.find(id);
        if (found == lookup_edges.end()) return;
        Edge e = edges[found->second];
        Node* n1 = getNode(e.node_id1);
        if (n1)
        {
            for (auto it = n1->edge_ids.begin(); it != n1->edge_ids.end(); it++)
            {
                if (*it == e.id)
                {
                    n1->edge_ids.erase(it);
                    break;
                }
            }
            if (n1->type == Node::NODE_JUNCTION && n1->edge_ids.size() <= 2) n1->type = Node::NODE_BASIC;
        }
        Node* n2 = getNode(e.node_id2);
        if (n2)
        {
            for (auto it = n2->edge_ids.begin(); it != n2->edge_ids.end(); it++)
            {
                if (*it == e.id)
                {
                    n2->edge_ids.erase(it);
                    break;
                }
            }
            if (n2->type == Node::NODE_JUNCTION && n2->edge_ids.size() <= 2) n2->type = Node::NODE_BASIC;
        }
        edges.erase(edges.begin() + found->second);

        lookup_edges.clear();
        for (size_t idx = 0; idx < edges.size(); idx++)
        {
            lookup_edges.insert(std::make_pair(edges[idx].id, idx));
        }
        m_router_valid = false;
        m_map_rect_valid = false;
    }

    /**
     * Find a node using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    Node* getNode(ID id)
    {
        auto found = lookup_nodes.find(id);
        if (found == lookup_nodes.end()) return nullptr;
        return &nodes[found->second];
    }

    /**
     * Find a node using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    const Node* getNode(ID id) const
    {
        auto found = lookup_nodes.find(id);
        if (found == lookup_nodes.end()) return nullptr;
        return &nodes[found->second];
    }

    /**
     * Find a nearest node from a given point (time complexity: O(|N|))
     * @param p A given point
     * @return A pointer to the found node (nullptr if not found)
     */
    Node* getNearestNode(const Point2& p);

    /**
     * Find a nearest node from a given point (time complexity: O(|N|))
     * @param p A given point
     * @return A pointer to the found node (nullptr if not found)
     */
    const Node* getNearestNode(const Point2& p) const;

    /**
     * Find a nearest node from a given Node (time complexity: O(|N|))
     * @param p A given node ID
     * @return A pointer to the found node (nullptr if not found)
     */
    Node* getNearestNode(const ID id);

    /**
     * Find a nearest node from a given Node (time complexity: O(|N|))
     * @param p A given node ID
     * @return A pointer to the found node (nullptr if not found)
     */
    const Node* getNearestNode(const ID id) const;

    /**
     * Find nodes within a search radius from a point (time complexity: O(|N|))
     * @param p A given point
     * @param search_radius A given search radius
     * @return A list of found node (empty list if no node found)
     */
    std::vector<Node*> getNearNodes(const Point2& p, double search_radius);

    /**
     * Find nodes within a search radius from a point (time complexity: O(|N|))
     * @param p A given point
     * @param search_radius A given search radius
     * @return A list of found node (empty list if no node found)
     */
    std::vector<const Node*> getNearNodes(const Point2& p, double search_radius) const;

    /**
     * Find nodes connected to a given node within a search radius (time complexity: O(1))
     * @param node A given node
     * @param search_radius A given search radius
     * @return A list of found node (empty list if no node found)
     */
    std::vector<Node*> getConnectedNearNodes(Node* node, double search_radius);

    /**
     * Find nodes connected to a given node within a search radius (time complexity: O(1))
     * @param node A given node
     * @param search_radius A given search radius
     * @return A list of found node (empty list if no node found)
     */
    std::vector<const Node*> getConnectedNearNodes(const Node* node, double search_radius) const;

    /**
     * Find a node connected from a given node with a given edge (time complexity: O(1))
     * @param from Given reference node
     * @param edge_id Given edge ID
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    Node* getConnectedNode(const Node* from, ID edge_id)
    {
        if (from == nullptr) return nullptr;
        Edge* edge = getEdge(edge_id);
        if (edge == nullptr) return nullptr;
        if (edge->node_id1 == from->id) return getNode(edge->node_id2);
        if (edge->node_id2 == from->id) return getNode(edge->node_id1);
        return nullptr;
    }

    /**
     * Find a node connected from a given node with a given edge (time complexity: O(1))
     * @param from Given reference node
     * @param edge_id Given edge ID
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    const Node* getConnectedNode(const Node* from, ID edge_id) const
    {
        if (from == nullptr) return nullptr;
        const Edge* edge = getEdge(edge_id);
        if (edge == nullptr) return nullptr;
        if (edge->node_id1 == from->id) return getNode(edge->node_id2);
        if (edge->node_id2 == from->id) return getNode(edge->node_id1);
        return nullptr;
    }

    /**
     * Find an array index of a node using ID (time complexity: O(1))
     * @param node_id ID to search
     * @return An array index to the found node (-1 if not exist)
     */
    int getNodeIndex(ID node_id) const
    {
        auto found = lookup_nodes.find(node_id);
        if (found == lookup_nodes.end()) return -1;
        return (int)found->second;
    }

    /**
     * Find an edge using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found edge (`nullptr` if not exist)
     */
    Edge* getEdge(ID id)
    {
        assert(edges.size() == lookup_edges.size() && lookup_edges.count(id) <= 1); // Verify ID uniqueness (comment this line if you want speed-up in DEBUG mode)
        auto found = lookup_edges.find(id);
        if (found == lookup_edges.end()) return nullptr;
        return &edges[found->second];
    }

    /**
     * Find an edge using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found edge (`nullptr` if not exist)
     */
    const Edge* getEdge(ID id) const
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
    Edge* getEdge(ID from, ID to)
    {
        Node* from_ptr = getNode(from);
        if (from_ptr == nullptr) return nullptr;
        for (auto eid = from_ptr->edge_ids.begin(); eid != from_ptr->edge_ids.end(); eid++)
        {
			Edge* edge = getEdge(*eid);
            if (edge == nullptr) continue;
            if (edge->node_id1 == from && edge->node_id2 == to || !edge->directed && edge->node_id1 == to && edge->node_id2 == from) return edge;
        }
        return nullptr;
    }

    /**
     * Find an edge using ID (time complexity: O(|E|))
     * @param from ID of the start node
     * @param to ID of the destination node
     * @return A pointer to the found edge (`nullptr` if not exist)
     */
    const Edge* getEdge(ID from, ID to) const
    {
        const Node* from_ptr = getNode(from);
        if (from_ptr == nullptr) return nullptr;
        for (auto eid = from_ptr->edge_ids.begin(); eid != from_ptr->edge_ids.end(); eid++)
        {
            const Edge* edge = getEdge(*eid);
            if (edge == nullptr) continue;
            if (edge->node_id1 == from && edge->node_id2 == to || !edge->directed && edge->node_id1 == to && edge->node_id2 == from) return edge;
        }
        return nullptr;
    }

    /**
     * Find an edge using the edge's index (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param edge_idx The edge's index
     * @return A pointer to the found edge (nullptr if not exist)
     * @see getEdge
     */
    Edge* getEdge(const Node* from, int edge_idx)
    {
        if (from == nullptr) return nullptr;
        if (edge_idx >= from->edge_ids.size()) return nullptr;
        return getEdge(from->edge_ids[edge_idx]);
    }

    /**
     * Find an edge using the edge's index (time complexity: O(|E|))
     * @param from A pointer to the start node
     * @param edge_idx The edge's index
     * @return A pointer to the found edge (nullptr if not exist)
     * @see getEdge
     */
    const Edge* getEdge(const Node* from, int edge_idx) const
    {
        if (from == nullptr) return nullptr;
        if (edge_idx >= from->edge_ids.size()) return nullptr;
        return getEdge(from->edge_ids[edge_idx]);
    }

    /**
     * Find the edge index of an edge from a node
     * @param node A given node
     * @param edge_id Edge ID to search
     * @return Edge index of the edge (-1 if the edge is not in the node)
     */
    int getEdgeIndex(const Node* node, ID edge_id) const
    {
        if (node == nullptr) return -1;
        for (int i = 0; i < (int)node->edge_ids.size(); i++)
        {
            if (node->edge_ids[i] == edge_id) return i;
        }
        return -1;
    }

    /**
     * Find a nearest edge from a given point (time complexity: O(|E|))
     * @param p A given point
     * @param nearest_edge_point An edge point that is nearest to p
     * @return A pointer to the found edge (nullptr if not found)
     */
    Edge* getNearestEdge(const Point2& p, Point2& nearest_edge_point);

    /**
     * Find a nearest edge from a given point (time complexity: O(|E|))
     * @param p A given point
     * @param nearest_edge_point An edge point that is nearest to p
     * @return A pointer to the found edge (nullptr if not found)
     */
    const Edge* getNearestEdge(const Point2& p, Point2& nearest_edge_point) const;

    /**
     * Find a nearest map-projected pose of a given pose (time complexity: O(|E|))
     * @param pose_m A given metric pose
     * @return The found map pose
     */
    Pose2 getNearestMapPose(const Pose2& pose_m) const;

    /**
     * Find a nearest path-projected pose for a given pose and path (time complexity: O(|E|))
     * @param path A given path
     * @param pose_m A given metric pose
     * @return The found path-projected pose
     */
    static Pose2 getNearestPathPose(const Path& path, const Pose2& pose_m);

    /**
	 * Add a POI (time complexity: O(1))
	 * @param poi POI to add
     * @return True if successful (false if failed)
     */
	bool addPOI(const POI& poi)
	{
		size_t poi_idx = pois.size();
		auto result = lookup_pois.insert(std::make_pair(poi.id, poi_idx));
        if (!result.second) return false;
        pois.push_back(poi);

        // update poi names lookup
        auto found = lookup_poi_names.find(poi.name);
        if (found == lookup_poi_names.end())
        {
            size_t data_idx = poi_names_data.size();
            lookup_poi_names.insert(std::make_pair(poi.name, data_idx));
            std::vector<size_t> data;
            data.push_back(poi_idx);
            poi_names_data.push_back(data);
        }
        else
        {
            poi_names_data[found->second].push_back(poi_idx);
        }
        return true;
	}

    /**
     * Delete a POI (time complexity: O(1))
     * @param id POI ID to delete
     */
    void deletePOI(ID id)
    {
        auto found = lookup_pois.find(id);
        if (found == lookup_pois.end()) return;

        size_t poi_idx = found->second;
        POI poi = pois[found->second];
        pois.erase(pois.begin() + found->second);

        // update poi lookup
        lookup_pois.clear();
        for (auto idx = 0; idx < pois.size(); idx++)
        {
            lookup_pois.insert(std::make_pair(pois[idx].id, idx));
        }

        // update poi names lookup
        lookup_poi_names.clear();
        poi_names_data.clear();
        for (auto idx = 0; idx < pois.size(); idx++)
        {
            auto found = lookup_poi_names.find(pois[idx].name);
            if (found == lookup_poi_names.end())
            {
                size_t data_idx = poi_names_data.size();
                lookup_poi_names.insert(std::make_pair(pois[idx].name, data_idx));
                std::vector<size_t> data;
                data.push_back(idx);
                poi_names_data.push_back(data);
            }
            else
            {
                poi_names_data[found->second].push_back(idx);
            }
        }
    }

    /**
     * Find a POI using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found POI (`nullptr` if not exist)
     */
    POI* getPOI(ID id)
    {
        auto found = lookup_pois.find(id);
        if (found == lookup_pois.end()) return nullptr;
        return &pois[found->second];
    }

    /**
     * Find a POI using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found POI (`nullptr` if not exist)
     */
    const POI* getPOI(ID id) const
    {
        auto found = lookup_pois.find(id);
        if (found == lookup_pois.end()) return nullptr;
        return &pois[found->second];
    }

    /**
     * Find POIs using name (time complexity: O(1))
     * @param name Name to search
     * @return A list of found POI pointers
     */
    std::vector<POI*> getPOI(std::wstring name)
    {
        auto found = lookup_poi_names.find(name);
        if (found == lookup_poi_names.end()) return std::vector<POI*>();
        std::vector<POI*> results;
        for (auto it = poi_names_data[found->second].begin(); it != poi_names_data[found->second].end(); it++)
        {
            results.push_back(&(pois[*it]));
        }
        return results;
    }

    /**
     * Find POIs using name within a search radius from a point (time complexity: O(1))
     * @param name Name to search
     * @param p A given point
     * @param search_radius A given search radius
     * @param sorted If True, the POIs are sorted in ascending order w.r.t diatance from a given point p
     * @return A list of found POI pointers (`nullptr` if not exist)
     */
    std::vector<POI*> getPOI(std::wstring name, const Point2& p, double search_radius, bool sorted = false);

    /**
     * Find POIs within a search radius from a point (time complexity: O(|N|))
     * @param p A given point
     * @param search_radius A given search radius
     * @param sorted If True, the POIs are sorted in ascending order w.r.t diatance from a given point p
     * @return A list of found POIs (empty list if no POI found)
     */
    std::vector<POI*> getNearPOIs(const Point2& p, double search_radius, bool sorted = false);

    /**
     * Find POI names within a search radius from a point (time complexity: O(|N|))
     * @param p A given point
     * @param search_radius A given search radius
     * @return A list of names of found POIs (empty list if no POI found)
     */
    std::vector<std::wstring> getNearPOINames(const Point2& p, double search_radius);

    /**
     * Register an image to POI node
     * @param id ID of POI
     * @param image An image to register
     * @return True if successful (false if failed)
     */
    bool registerPOIImage(ID id, cv::Mat image);

    /**
     * Register an image to POI node
     * @param id ID of POI
     * @param image An image to register
     * @param image_name Assigned image name of the registered image
     * @return True if successful (false if failed)
     */
    bool registerPOIImage(ID id, cv::Mat image, std::string& image_name);

    /**
     * Get the number of registered POI images
     * @param id ID of POI node
     * @return The number of registered POI images
     */
    int getNumberOfRegisteredPOIImages(ID id) const;

    /**
     * Get images registered to a POI node
     * @param id ID of POI node
     * @return A list of images registered to the POI
     */
    std::vector<cv::Mat> getRegisteredPOIImages(ID id) const;

    /**
     * Get i-th image registered to a POI node
     * @param id ID of POI node
     * @param index index of POI image
     * @return The i-th POI image
     */
    cv::Mat getRegisteredPOIImage(ID id, int index) const;

    /**
     * Get image names registered to a POI node
     * @param id ID of POI node
     * @return A list of image names registered to the POI
     */
    std::vector<std::string> getNameOfRegisteredPOIImages(ID id) const;

    /**
     * Remove all the images registered to a POI node
     * @param id ID of POI node
     */
    void clearRegisteredPOIImages(ID id);

    /**
	 * Add a Street-view (time complexity: O(1))
	 * @param view Street-view to add
	 * @return A index of the added Street-view
	 */
	bool addView(const StreetView& view)
	{
        size_t view_idx = views.size();
        auto result = lookup_views.insert(std::make_pair(view.id, view_idx));
        if (!result.second) return false;
        views.push_back(view);
		return true;
	}

    /**
     * Delete a Street-view (time complexity: O(1))
     * @param id Street-view ID to delete
     */
    void deleteView(ID id)
    {
        auto found = lookup_views.find(id);
        if (found == lookup_views.end()) return;
        views.erase(views.begin() + found->second);

        lookup_views.clear();
        for (size_t idx = 0; idx < views.size(); idx++)
        {
            lookup_views.insert(std::make_pair(views[idx].id, idx));
        }
    }

    /**
     * Find a streetview using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    StreetView* getView(ID id)
    {
        auto found = lookup_views.find(id);
        if (found == lookup_views.end()) return nullptr;
        return &views[found->second];
    }

    /**
     * Find a streetview using ID (time complexity: O(1))
     * @param id ID to search
     * @return A pointer to the found node (`nullptr` if not exist)
     */
    const StreetView* getView(ID id) const
    {
        assert(views.size() == lookup_views.size() && lookup_views.count(id) <= 1); // Verify ID uniqueness (comment this line if you want speed-up in DEBUG mode)
        auto found = lookup_views.find(id);
        if (found == lookup_views.end()) return nullptr;
        return &views[found->second];
    }

    /**
     * Find streetviews within a search radius from a point (time complexity: O(|N|))
     * @param p A given point
     * @param search_radius A given search radius
     * @param sorted If True, the Views are sorted in ascending order w.r.t diatance from a given point p
     * @return A list of found streetviews (empty list if no streetview found)
     */
    std::vector<StreetView*> getNearViews(const Point2& p, double search_radius, bool sorted = false);

    /**
     * Find a shortest path between two points using Dijkstra's shortest path algorithm (time complexity: O(|N|^2))
     * @param from Start position
     * @param to Destination position
     * @param path The found path
     * @return True if successful (false if failed)
     */
    bool getPath(Point2 from, Point2 to, Path& path);

    /**
     * Find a shortest path between two nodes using Dijkstra's shortest path algorithm (time complexity: O(|N|^2))
     * @param from Start node
     * @param to Destination node
     * @param path The found path
     * @return True if successful (false if failed)
     */
    bool getPath(ID from, ID to, Path& path);

    /**
     * Check whether this map is empty or not
     * @param check_only_topomap If True is given, check only topology map (default value)
     * @return True if empty (true) or not (false)
     */
    bool isEmpty(bool check_only_topomap = true) const
    {
        if(check_only_topomap) return (countNodes() <= 0);
        return (countNodes() <= 0 && countPOIs() <= 0 && countViews() <= 0);
    }

    /**
     * Check whether POI map of this map is empty or not
     * @return True if empty (true) or not (false)
     */
    bool isEmptyPOIMap() const { return (countPOIs() <= 0); }

    /**
     * Check whether StreetView map of this map is empty or not
     * @return True if empty (true) or not (false)
     */
    bool isEmptyViewMap() const { return (countViews() <= 0); }

    /** Reserve memory in advance for preventing memory reallocation */
    void reserveMemory(int max_nodes = 17149, int max_edges = 24314, int max_pois = 22217, int max_views = 55513)
    {
        nodes.reserve(max_nodes);
        edges.reserve(max_edges);
        pois.reserve(max_pois);
        poi_names_data.reserve(max_pois);
        views.reserve(max_views);
    }

    /**
     * Count the number of all nodes (time complexity: O(1))
     * @return The number of nodes
     */
    int countNodes() const { return (int)nodes.size(); }

    /**
     * Count the number of edges starting from the given node (time complexity: O(1))
     * @param node_id ID of the node
     * @return The number of edges
     */
    int countEdges(ID node_id) const
    {
        auto found = lookup_nodes.find(node_id);
        if (found == lookup_nodes.end()) return 0;
        return (int)nodes[found->second].edge_ids.size();
    }

    /**
     * Count the number of edges starting from the given node (time complexity: O(1))
     * @param node A pointer to the node
     * @return The number of edges
     */
    int countEdges(const Node* node) const
    {
        if (node == nullptr) return 0;
        return (int)node->edge_ids.size();
    }

    /**
     * Count the number of all nodes (time complexity: O(1))
     * @return The number of nodes
     */
    int countEdges() const { return (int)edges.size(); }

    /**
     * Count the number of all nodes (time complexity: O(1))
     * @return The number of nodes
     */
    int countPOIs() const { return (int)pois.size(); }

    /**
     * Count the number of all nodes (time complexity: O(1))
     * @return The number of nodes
     */
    int countViews() const { return (int)views.size(); }

    /**
     * Get an iterator of the first node in this graph (time complexity: O(1))
     * @return An iterator of the first node
     * @see getTailNode
     */
    NodeItr getHeadNode() { return nodes.begin(); }

    /**
     * Get an iterator of the ending node in this graph (time complexity: O(1))
     * @return An iterator of the ending node
     * @see getHeadNode
     */
    NodeItr getTailNode() { return nodes.end(); }

    /**
     * Get an iterator of the first edge in this graph (time complexity: O(1))
     * @return An iterator of the first edge
     * @see getTailEdge
     */
    EdgeItr getHeadEdge() { return edges.begin(); }

    /**
     * Get an iterator of the ending edge in this graph (time complexity: O(1))
     * @return An iterator of the ending edge
     * @see getHeadEdge
     */
    EdgeItr getTailEdge() { return edges.end(); }

    /**
     * Get an iterator of the first edge of a node (time complexity: O(1))
     * @return An iterator of the first edge
     * @see getTailEdgeID
     */
    IDItr getHeadEdgeID(Node* node) { return node->edge_ids.begin(); }

    /**
     * Get an iterator of the ending edge of a node (time complexity: O(1))
     * @return An iterator of the ending edge
     * @see getHeadEdgeID
     */
    IDItr getTailEdgeID(Node* node) { return node->edge_ids.end(); }

    /**
     * Get an iterator of the first POI in this graph (time complexity: O(1))
     * @return An iterator of the first POI
     * @see getTailPOI
     */
    POIItr getHeadPOI() { return pois.begin(); }

    /**
     * Get an iterator of the ending POI in this graph (time complexity: O(1))
     * @return An iterator of the ending POI
     * @see getHeadPOI
     */
    POIItr getTailPOI() { return pois.end(); }

    /**
     * Get an iterator of the first StreetView in this graph (time complexity: O(1))
     * @return An iterator of the first StreetView
     * @see getTailView
     */
    StreetViewItr getHeadView() { return views.begin(); }

    /**
     * Get an iterator of the ending StreetView in this graph (time complexity: O(1))
     * @return An iterator of the ending StreetView
     * @see getHeadView
     */
    StreetViewItr getTailView() { return views.end(); }

    /**
     * Get an iterator of the first node in this graph (time complexity: O(1))
     * @return An iterator of the first node
     * @see getTailNode
     */
    NodeItrConst getHeadNodeConst() const { return nodes.cbegin(); }

    /**
     * Get an iterator of the ending node in this graph (time complexity: O(1))
     * @return An iterator of the ending node
     * @see getHeadNode
     */
    NodeItrConst getTailNodeConst() const { return nodes.cend(); }

    /**
     * Get an iterator of the first edge in this graph (time complexity: O(1))
     * @return An iterator of the first edge
     * @see getTailEdge
     */
    EdgeItrConst getHeadEdgeConst() const { return edges.cbegin(); }

    /**
     * Get an iterator of the ending edge in this graph (time complexity: O(1))
     * @return An iterator of the ending edge
     * @see getHeadEdge
     */
    EdgeItrConst getTailEdgeConst() const { return edges.cend(); }

    /**
     * Get an iterator of the first edge of a node (time complexity: O(1))
     * @return An iterator of the first edge
     * @see getTailEdgeID
     */
    IDItrConst getHeadEdgeIDConst(Node* node) const { return node->edge_ids.cbegin(); }

    /**
     * Get an iterator of the ending edge of a node (time complexity: O(1))
     * @return An iterator of the ending edge
     * @see getHeadEdgeID
     */
    IDItrConst getTailEdgeIDConst(Node* node) const { return node->edge_ids.cend(); }

    /**
     * Get an iterator of the first POI in this graph (time complexity: O(1))
     * @return An iterator of the first POI
     * @see getTailPOI
     */
    POIItrConst getHeadPOIConst() const { return pois.cbegin(); }

    /**
     * Get an iterator of the ending POI in this graph (time complexity: O(1))
     * @return An iterator of the ending POI
     * @see getHeadPOI
     */
    POIItrConst getTailPOIConst() const { return pois.cend(); }

    /**
     * Get an iterator of the first StreetView in this graph (time complexity: O(1))
     * @return An iterator of the first StreetView
     * @see getTailView
     */
    StreetViewItrConst getHeadViewConst() const { return views.cbegin(); }

    /**
     * Get an iterator of the ending StreetView in this graph (time complexity: O(1))
     * @return An iterator of the ending StreetView
     * @see getHeadView
     */
    StreetViewItrConst getTailViewConst() const { return views.cend(); }

    /**
     * Get a bounding rectangle of the area the map covers
     * @return Bounding rectangle of the positions of the map nodes
     */
    cv::Rect2d getMapBoundingRect()
    {
        if(m_map_rect_valid) return m_map_rect;
        if (isEmpty()) return cv::Rect2d();

        double min_x = getHeadNode()->x;
        double max_x = getHeadNode()->x;
        double min_y = getHeadNode()->y;
        double max_y = getHeadNode()->y;
        for (auto it = getHeadNode(); it != getTailNode(); it++)
        {
            if (it->x < min_x) min_x = it->x;
            if (it->x > max_x) max_x = it->x;
            if (it->y < min_y) min_y = it->y;
            if (it->y > max_y) max_y = it->y;
        }
        m_map_rect = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
        m_map_rect_valid = true;

        return m_map_rect;
    }

    bool findID(std::vector<ID> ids, ID eid)
    {
        for (std::vector<ID>::iterator iter = ids.begin(); iter != ids.end(); iter++)
        {
            if (*iter == eid)
                return true;
        }
        return false;
    }

    int getDegree(Point2 p1, Point2 p2, Point2 p3)
    {
        Point2 v1 = p2 - p1;
        Point2 v2 = p3 - p2;
        if (norm(v1) <= 0 || norm(v2) <= 0) return 0;

        double ang_rad = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));
        if (v1.cross(v2) < 0) ang_rad = -ang_rad;
        double ang_deg = ang_rad * 180 / CV_PI;

        return (int)ang_deg;
    }

    void setEdgeLR(const Edge* edge, int lr_side)
    {
        Edge* cur_edge = getEdge(edge->id);
        cur_edge->lr_side = lr_side;
    }

    void addEdgeLR()
    {
        //lrpose
        std::vector<ID> finished_edges;
        for (auto e = getHeadEdgeConst(); e != getTailEdgeConst(); e++)
        {
            if (e->type == Edge::EDGE_CROSSWALK)
            {
                ID cross_edge1_id = e->id;
                Node * node1, * node2, * node3;
                for (int i = 0; i < 2; i++)
                {
                    //searching the other node of the crosswalk
                    if (i != 0)
                    {
                        node1 = getNode(e->node_id2);
                        node2 = getNode(e->node_id1);
                    }
                    else
                    {
                        node1 = getNode(e->node_id1);
                        node2 = getNode(e->node_id2);
                    }

                    ID node3_id;
                    Edge* edge1, * edge2;
                    std::vector<ID> candidate_edges;

                    //search first sidewalk connected to the crosswalk
                    std::vector<ID> cross_conn_edges = node2->edge_ids;
                    for (std::vector<ID>::iterator side1id = cross_conn_edges.begin(); side1id != cross_conn_edges.end(); side1id++)
                    {
                        if (*side1id != cross_edge1_id)
                        {
                            //get second edge
                            edge2 = getEdge(*side1id);
                            if (findID(finished_edges, edge2->id))
                                continue;

                            //get third node
                            node3_id = (edge2->node_id1 == node2->id) ? edge2->node_id2 : edge2->node_id1;
                            node3 = getNode(node3_id);
                            int edges_deg = getDegree(Point2(node1->x, node1->y), Point2(node2->x, node2->y), Point2(node3->x, node3->y));

                            //check degree
                            if (edges_deg > 30 && edges_deg < 150)    //I'm on the right of the road
                            {
                                //save direction
                                if (edge2->type == Edge::EDGE_SIDEWALK)
                                {
                                    //save direction
                                    int lr_side;
                                    if (edge2->node_id1 == node2->id)
                                        lr_side = Edge::LR_RIGHT;
                                    else
                                        lr_side = Edge::LR_LEFT;
                                    setEdgeLR(edge2, lr_side);
                                    finished_edges.push_back(edge2->id);
                                }

                                //continue to next sidewalk
                                node1 = node2;
                                node2 = node3;
                                edge1 = edge2;
                                bool bFlag = true; int count = 0;
                                while (bFlag && count < 20)
                                {
                                    bFlag = false; count++;
                                    candidate_edges = node2->edge_ids;
                                    for (std::vector<ID>::iterator side2 = candidate_edges.begin(); side2 != candidate_edges.end(); side2++)
                                    {
                                        if (*side2 == edge1->id)//pass edge1
                                            continue;

                                        edge2 = getEdge(*side2);
                                        if (findID(finished_edges, edge2->id))//pass already done
                                            continue;

                                        node3_id = (edge2->node_id1 == node2->id) ? edge2->node_id2 : edge2->node_id1;
                                        node3 = getNode(node3_id);
                                        int edges_deg = getDegree(Point2(node1->x, node1->y), Point2(node2->x, node2->y), Point2(node3->x, node3->y));

                                        //the sidewalk is connected to straight
                                        if (edges_deg > -30 && edges_deg < 30)  //straight line
                                        {
                                            //save direction
                                            if (edge2->type == Edge::EDGE_SIDEWALK)
                                            {
                                                int lr_side;
                                                if (edge2->node_id1 == node2->id)
                                                    lr_side = Edge::LR_RIGHT;
                                                else
                                                    lr_side = Edge::LR_LEFT;
                                                setEdgeLR(edge2, lr_side);
                                                finished_edges.push_back(edge2->id);
                                            }

                                            node1 = node2;
                                            node2 = node3;
                                            edge1 = edge2;
                                            bFlag = true;

                                            break;  //end for{}
                                        }
                                        else
                                            bFlag = false;
                                    }
                                } //end while()
                            }//end if (edges_deg > 30 && edges_deg < 150)  //I'm on the right of the road

                            else if (edges_deg > -150 && edges_deg < -30) //I'm on the left side of the road
                            {
                                //save direction
                                if (edge2->type == Edge::EDGE_SIDEWALK)
                                {
                                    int lr_side;
                                    if (edge2->node_id1 == node2->id)
                                        lr_side = Edge::LR_LEFT;
                                    else
                                        lr_side = Edge::LR_RIGHT;
                                    setEdgeLR(edge2, lr_side);
                                    finished_edges.push_back(edge2->id);
                                }

                                //continue to next sidewalk
                                node1 = node2;
                                node2 = node3;
                                edge1 = edge2;
                                bool bFlag = true; int count = 0;
                                while (bFlag && count < 20)
                                {
                                    bFlag = false; count++;
                                    candidate_edges = node2->edge_ids;
                                    for (std::vector<ID>::iterator side2 = candidate_edges.begin(); side2 != candidate_edges.end(); side2++)
                                    {
                                        if (*side2 == edge1->id)//pass edge1
                                            continue;

                                        edge2 = getEdge(*side2);
                                        if (findID(finished_edges, edge2->id))//pass already done
                                            continue;

                                        node3_id = (edge2->node_id1 == node2->id) ? edge2->node_id2 : edge2->node_id1;
                                        node3 = getNode(node3_id);
                                        int edges_deg = getDegree(Point2(node1->x, node1->y), Point2(node2->x, node2->y), Point2(node3->x, node3->y));

                                        //the sidewalk is connected to straight
                                        if (edges_deg > -30 && edges_deg < 30)  //straight line
                                        {
                                            //save direction
                                            if (edge2->type == Edge::EDGE_SIDEWALK)
                                            {
                                                //save direction
                                                int lr_side;
                                                if (edge2->node_id1 == node2->id)
                                                    lr_side = Edge::LR_LEFT;
                                                else
                                                    lr_side = Edge::LR_RIGHT;
                                                setEdgeLR(edge2, lr_side);
                                                finished_edges.push_back(edge2->id);
                                            }
                                            node1 = node2;
                                            node2 = node3;
                                            edge1 = edge2;
                                            bFlag = true;
                                            break;  //end for{}
                                        }
                                        else
                                            bFlag = false;
                                    }
                                } //end while()
                            }//end else if(edges_deg > -150 && edges_deg < -30) //I'm on the left of the road
                        }//end of if (*iter != edge1_id)
                    }//end of for (vector<ID>::iterator iter = candidate_edges.begin(); iter != candidate_edges.end(); iter++)
                }//end for (int i = 0; i < 2; i++)
            }//end if (e->type == Edge::EDGE_CROSSWALK)
        }//end for (auto e = map.getHeadEdgeConst(); e != map.getTailEdgeConst(); e++)
    }

	/**
	 * Get the union of two Map sets
	 * @param set2 The given Map set of this union set
	 */
	void setUnion(const Map& set2) 
	{
		Map set1 = *this;
		
		for (auto node = set2.lookup_nodes.begin(); node != set2.lookup_nodes.end(); ++node)
		{
			auto result = set1.lookup_nodes.insert(std::make_pair(node->first, set1.nodes.size()));
			if( result.second )
				set1.nodes.push_back(set2.nodes[node->second]);
		}

		for (auto edge = set2.lookup_edges.begin(); edge != set2.lookup_edges.end(); ++edge)
		{
			auto result = set1.lookup_edges.insert(std::make_pair(edge->first, set1.edges.size()));
			if (result.second)
				set1.edges.push_back(set2.edges[edge->second]);
		}

		for (auto poi = set2.lookup_pois.begin(); poi != set2.lookup_pois.end(); ++poi)
		{
			auto result = set1.lookup_pois.insert(std::make_pair(poi->first, set1.pois.size()));
			if (result.second)
				set1.pois.push_back(set2.pois[poi->second]);
		}

		for (auto view = set2.lookup_views.begin(); view != set2.lookup_views.end(); ++view)
		{
			auto result = set1.lookup_views.insert(std::make_pair(view->first, set1.views.size()));
			if (result.second)
				set1.views.push_back(set2.views[view->second]);
		}

		*this = set1;
        m_router_valid = false;
        m_map_rect_valid = false;
    }

    /**
     * Copy this to the other map (time complexity: O(|N||E|))
     * @param dest A pointer to the other map
     * @return True if successful (false if failed)
     */
    bool copyTo(Map* dest) const;

    /**
     * Overriding the assignment operator
     * @param rhs A map in the right-hand side
     * @return This object
     */
    Map& operator=(const Map& rhs)
    {
        rhs.copyTo(this);
        m_router_valid = false;
        m_map_rect_valid = false;
        return *this;
    }

    /**
     * Remove all nodes and edges and routing variables
     */
    void removeTopoMap()
    {
        nodes.clear();
        edges.clear();
        lookup_nodes.clear();
        lookup_edges.clear();
        resetRouterVariables();
        m_map_rect_valid = false;
        m_map_rect = cv::Rect2d();
    }

    /**
     * Remove poi map data
     */
    void removePOIMap()
    {
        pois.clear();
        lookup_pois.clear();
        poi_names_data.clear();
        lookup_poi_names.clear();
    }

    /**
     * Remove streetview map data
     */
    void removeViewMap()
    {
        views.clear();
        lookup_views.clear();
    }

    /**
     * Remove all data
     */
    void removeAll()
    {
        removeTopoMap();
        removePOIMap();
        removeViewMap();
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

    /** A hash table for finding nodes <ID,index> */
    std::map<ID, size_t> lookup_nodes;

    /** A hash table for finding edges <ID,index> */
    std::map<ID, size_t> lookup_edges;

	/** A hash table for finding POIs <ID,index> */
	std::map<ID, size_t> lookup_pois;

    /** A hash table for finding POI names <name,index> */
    std::map<std::wstring, size_t> lookup_poi_names;
    std::vector<std::vector<size_t>> poi_names_data;

	/** A hash table for finding Street-views */
	std::map<ID, size_t> lookup_views;

    /** A bouding rectangle of the area the map covers (Uint: [m]) */
    cv::Rect2d m_map_rect;
    bool m_map_rect_valid = false;

    /** Internal variables for router */
    bool m_router_valid = false;
    const Node* m_dest_node = nullptr;
    std::vector<double> m_distance;
    std::vector<bool> m_found;
    std::vector<int> m_next_idx;

    /** Internal api's for router */
    bool initializeRouterVariables(const Node* dest_node);
    void resetRouterVariables();
    int chooseBestUnvisited(const std::vector<double>& distance, const std::vector<bool>& found);
};

} // End of 'dg'

#endif // End of '__MAP__'
