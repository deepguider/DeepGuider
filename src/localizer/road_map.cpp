#include "road_map.hpp"

#define ROAD_MAP_BUF_SIZE               (1024)

namespace dg
{

// 'RoadMap' class
bool RoadMap::load(const char* filename)
{
    removeAll();

    FILE* fid = fopen(filename, "rt");
    if (fid == nullptr) return false;

    char buffer[ROAD_MAP_BUF_SIZE];
    while (!feof(fid))
    {
        if (fgets(buffer, ROAD_MAP_BUF_SIZE, fid) == nullptr) break;
        char* token;
        if ((token = strtok(buffer, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        if (token[0] == 'N' || token[0] == 'n')
        {
            // Read nodes
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double x = strtod(token, nullptr);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double y = strtod(token, nullptr);

            if (addNode(Point2ID(id, x, y)) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        }
        else if (token[0] == 'E' || token[0] == 'e')
        {
            // Read edges
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id1 = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            ID id2 = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) goto ROADMAP_LOADMAP_FAIL;
            double cost = strtod(token, nullptr);
            Node* node1 = getNode(Point2ID(id1));
            Node* node2 = getNode(Point2ID(id2));
            if (node1 == nullptr || node2 == nullptr) goto ROADMAP_LOADMAP_FAIL;

            if (cost < 0)
            {
                double dx = node2->data.x - node1->data.x;
                double dy = node2->data.y - node1->data.y;
                cost = sqrt(dx * dx + dy * dy);
            }
            if (addEdge(node1, node2, cost) == nullptr) goto ROADMAP_LOADMAP_FAIL;
        }
    }
    fclose(fid);
    return true;

ROADMAP_LOADMAP_FAIL:
    removeAll();
    fclose(fid);
    return false;
}

bool RoadMap::save(const char* filename)
{
    if (isEmpty()) return false;

    FILE* file = fopen(filename, "wt");
    if (file == nullptr) return false;
    fprintf(file, "# NODE, ID, X [m], Y [m]\n");
    fprintf(file, "# EDGE, ID(from_ptr), ID(to_ptr), Cost\n");

    // Write nodes
    RoadMap::NodeItr itr_node;
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
        fprintf(file, "NODE, %zd, %lf, %lf\n", itr_node->data.id, itr_node->data.x, itr_node->data.y);

    // Write edges
    for (itr_node = getHeadNode(); itr_node != getTailNode(); itr_node++)
    {
        RoadMap::EdgeItr itr_edge;
        for (itr_edge = getHeadEdge(itr_node); itr_edge != getTailEdge(itr_node); itr_edge++)
            fprintf(file, "EDGE, %zd, %zd, %lf\n", itr_node->data.id, itr_edge->to->data.id, itr_edge->cost);
    }

    fclose(file);
    return true;
}

bool RoadMap::addRoad(Node* node1, Node* node2, double cost /*= -1.0*/)
{
    if (node1 == nullptr || node2 == nullptr) return false;

    if (cost < 0)
    {
        double dx = node1->data.x - node2->data.x;
        double dy = node1->data.y - node2->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    Edge* edge1 = DirectedGraph<Point2ID, double>::addEdge(node1, node2, cost);
    Edge* edge2 = DirectedGraph<Point2ID, double>::addEdge(node2, node1, cost);
    return (edge1 != nullptr) && (edge2 != nullptr);
}

RoadMap::Edge* RoadMap::addEdge(Node* from, Node* to, double cost /*= -1.0*/)
{
    if (from == nullptr || to == nullptr) return nullptr;

    if (cost < 0)
    {
        double dx = from->data.x - to->data.x;
        double dy = from->data.y - to->data.y;
        cost = sqrt(dx * dx + dy * dy);
    }
    return DirectedGraph<Point2ID, double>::addEdge(from, to, cost);
}

RoadMap::Edge* RoadMap::getEdge(ID from, ID to)
{
    Node* fnode = getNode(from);
    if (fnode == nullptr) return nullptr;
    for (EdgeItr edge_itr = getHeadEdge(fnode); edge_itr != getTailEdge(fnode); edge_itr++)
        if (edge_itr->to->data.id == to) return &(*edge_itr);
    return nullptr;
}

RoadMap::Edge* RoadMap::getEdge(Node* from, int edge_idx)
{
    if (from == nullptr) return nullptr;
    int edge_cnt = 0;
    for (auto edge_itr = getHeadEdge(from); edge_itr != getTailEdge(from); edge_itr++, edge_cnt++)
        if (edge_cnt == edge_idx) return &(*edge_itr);
    return nullptr;
}

bool RoadMap::copyTo(RoadMap* dest) const
{
    if (dest == nullptr) return false;

    dest->removeAll();
    for (NodeItrConst node = getHeadNodeConst(); node != getTailNodeConst(); node++)
        dest->addNode(node->data);
    for (NodeItrConst from = getHeadNodeConst(); from != getTailNodeConst(); from++)
        for (EdgeItrConst edge = getHeadEdgeConst(from); edge != getTailEdgeConst(from); edge++)
            dest->addEdge(dest->getNode(from->data), dest->getNode(edge->to->data), edge->cost);
    return true;
}

bool RoadMap::getPath(Point2 p1, Point2 p2, Path& path)
{
    // reset path
    path.pts.clear();

    // get nearest edge
    ID p1_nid1, p1_nid2;
    Point2 p1_ep = getNearestEdgePoint(p1, p1_nid1, p1_nid2);
    if (p1_nid1 == 0 || p1_nid2 == 0) return false;
    Edge* edge1 = getEdge(p1_nid1, p1_nid2);

    ID p2_nid1, p2_nid2;
    Point2 p2_ep = getNearestEdgePoint(p2, p2_nid1, p2_nid2);
    if (p2_nid1 == 0 || p2_nid2 == 0) return false;
    Edge* edge2 = getEdge(p2_nid1, p2_nid2);

    // select close node on each edge as the start & end path node
    Node* from = getNode(p1_nid1);
    double d1 = norm(p1_ep - from->data);
    if (d1 > edge1->cost / 2)
    {
        from = getNode(p1_nid2);
        d1 = edge1->cost - d1;
    }

    Node* to = getNode(p2_nid1);
    double d2 = norm(p2_ep - to->data);
    if (d2 > edge2->cost / 2)
    {
        to = getNode(p2_nid2);
        d2 = edge2->cost - d2;
    }

    if (from == nullptr || to == nullptr) return false;
    int from_idx = getNodeIndex(from->data.id);
    int to_idx = getNodeIndex(to->data.id);

    // initlize temporal variables
    initRoutingVariables(to, to_idx);

    // bread-first search
    int nNodes = (int)countNodes();
    while (!m_found[from_idx])
    {
        int best_ni = choose_best_unvisited(m_distance, m_found);
        if (best_ni < 0) break;
        m_found[best_ni] = true;
        Node* best = m_indexed_node_lookup[best_ni];
        if (best == nullptr) break;

        int j = 0;
        for (auto n = getHeadNode(); n != getTailNode(); j++, n++)
        {
            if (m_found[j] == false)
            {
                Edge* edge = getEdge(n->data.id, best->data.id);
                if (edge && (m_distance[best_ni] + edge->cost < m_distance[j]))
                {
                    m_distance[j] = m_distance[best_ni] + edge->cost;
                    m_next_idx[j] = best_ni;
                }
            }
        }
    }
    if (!m_found[from_idx]) return false;

    // build path
    for (auto idx = from_idx; idx != -1; idx = m_next_idx[idx])
    {
        Node* node = m_indexed_node_lookup[idx];
        if (node == nullptr) break;
        path.pts.push_back(PathElement(node->data.id, 0));
        if (idx == to_idx) break;
    }
    if (path.pts.front().node_id != from->data.id || path.pts.back().node_id != to->data.id) return false;

    // check & remove abnormal turn-around path (in case projected edge point is not node point)
    if ((int)path.pts.size() > 1 && d1 > DBL_EPSILON)
    {
        if (path.pts[1].node_id == p1_nid1 || path.pts[1].node_id == p1_nid2) path.pts.erase(path.pts.begin());            
    }
    if ((int)path.pts.size() > 1 && d2 > DBL_EPSILON)
    {
        int npath = (int)path.pts.size();
        if (path.pts[npath - 2].node_id == p2_nid1 || path.pts[npath - 2].node_id == p2_nid2) path.pts.pop_back();
    }

    return true;
}

bool RoadMap::getRoadPath(Point2 p1, Point2 p2, RoadPath& path)
{
    // reset path
    path.pts.clear();

    // get nearest edge
    ID p1_nid1, p1_nid2;
    Point2 p1_ep = getNearestEdgePoint(p1, p1_nid1, p1_nid2);
    if (p1_nid1 == 0 || p1_nid2 == 0) return false;
    Edge* edge1 = getEdge(p1_nid1, p1_nid2);

    ID p2_nid1, p2_nid2;
    Point2 p2_ep = getNearestEdgePoint(p2, p2_nid1, p2_nid2);
    if (p2_nid1 == 0 || p2_nid2 == 0) return false;
    Edge* edge2 = getEdge(p2_nid1, p2_nid2);

    // select close node on each edge as the start & end path node
    Node* from = getNode(p1_nid1);
    double ed1 = norm(p1_ep - from->data);
    if (ed1 > edge1->cost / 2)
    {
        from = getNode(p1_nid2);
        ed1 = norm(p1_ep - from->data);
    }

    Node* to = getNode(p2_nid1);
    double ed2 = norm(p2_ep - to->data);
    if (ed2 > edge2->cost / 2)
    {
        to = getNode(p2_nid2);
        ed2 = norm(p2_ep - to->data);
    }

    if (from == nullptr || to == nullptr) return false;
    int from_idx = getNodeIndex(from->data.id);
    int to_idx = getNodeIndex(to->data.id);

    // when p1 and p2 are projected on the same edge
    if (p1_nid1 == p2_nid1 && p1_nid2 == p2_nid2 || p1_nid1 == p2_nid2 && p1_nid2 == p2_nid1)
    {
        path.pts.push_back(Point2ID(0, p1));
        path.pts.push_back(Point2ID(0, p2));

        return true;
    }

    // initlize temporal variables
    initRoutingVariables(to, to_idx);

    // bread-first search
    int nNodes = (int)countNodes();
    while (!m_found[from_idx])
    {
        int best_ni = choose_best_unvisited(m_distance, m_found);
        if (best_ni < 0) break;
        m_found[best_ni] = true;
        Node* best = m_indexed_node_lookup[best_ni];
        if (best == nullptr) break;

        int j = 0;
        for (auto n = getHeadNode(); n != getTailNode(); j++, n++)
        {
            if (m_found[j] == false)
            {
                Edge* edge = getEdge(n->data.id, best->data.id);
                if (edge && (m_distance[best_ni] + edge->cost < m_distance[j]))
                {
                    m_distance[j] = m_distance[best_ni] + edge->cost;
                    m_next_idx[j] = best_ni;
                }
            }
        }
    }
    if (!m_found[from_idx]) return false;

    // build path
    for (auto idx = from_idx; idx != -1; idx = m_next_idx[idx])
    {
        Node* node = m_indexed_node_lookup[idx];
        if (node == nullptr) break;
        path.pts.push_back(node->data);
        if (idx == to_idx) break;
    }
    if (path.pts.front().id != from->data.id || path.pts.back().id != to->data.id) return false;

    // check & remove abnormal turn-around path (in case projected edge point is not node point)
    if ((int)path.pts.size() > 1 && ed1 > DBL_EPSILON)
    {
        if (path.pts[1].id == p1_nid1 || path.pts[1].id == p1_nid2) path.pts.erase(path.pts.begin());
    }
    if ((int)path.pts.size() > 1 && ed2 > DBL_EPSILON)
    {
        int npath = (int)path.pts.size();
        if (path.pts[npath - 2].id == p2_nid1 || path.pts[npath - 2].id == p2_nid2) path.pts.pop_back();
    }

    // add start_pos and dest_pos in path (in case they are not node point)
    double nd1 = norm(p1 - from->data);
    double nd2 = norm(p2 - to->data);
    if (nd1 > DBL_EPSILON) path.pts.insert(path.pts.begin(), Point2ID(0, p1));
    if (nd2 > DBL_EPSILON) path.pts.push_back(Point2ID(0, p2));

    return true;
}

bool RoadMap::initRoutingVariables(Node* dest_node, int dest_node_idx)
{
    if (!dest_node) return false;
    if (dest_node == m_dest_node) return true;

    int nNodes = (int)countNodes();
    m_distance.resize(nNodes);
    m_found.resize(nNodes);
    m_next_idx.resize(nNodes);
    m_indexed_node_lookup.resize(nNodes);

    int i = 0;
    for (auto n = getHeadNode(); n != getTailNode(); i++, n++)
    {
        Edge* edge = getEdge(n->data.id, dest_node->data.id);
        m_distance[i] = (edge) ? edge->cost : DBL_MAX;
        m_found[i] = false;
        m_next_idx[i] = (edge) ? dest_node_idx : -1;
        m_indexed_node_lookup[i] = &(*n);
    }

    m_distance[dest_node_idx] = 0;
    m_found[dest_node_idx] = true;
    m_next_idx[dest_node_idx] = -1;

    m_dest_node = dest_node;

    return true;
}

RoadMap::Node* RoadMap::getNearestNode(const Point2& p, int& node_idx)
{
    Node* node = nullptr;
    double min_d2 = DBL_MAX;
    node_idx = -1;
    int i = 0;
    for (auto n = getHeadNode(); n != getTailNode(); n++, i++)
    {
        double d2 = (p.x - n->data.x) * (p.x - n->data.x) + (p.y - n->data.y) * (p.y - n->data.y);
        if (d2 < min_d2)
        {
            min_d2 = d2;
            node_idx = i;
            node = &(*n);
        }
    }
    return node;
}

int RoadMap::getNodeIndex(ID node_id)
{
    int idx = 0;
    for (auto n = getHeadNode(); n != getTailNode(); n++, idx++)
    {
        if (n->data.id == node_id) return idx;
    }
    return -1;
}

int RoadMap::choose_best_unvisited(const std::vector<double>& distance, const std::vector<bool>& found)
{
    int n = (int)distance.size();
    double min_d = DBL_MAX;
    int min_i = -1;
    for (int i = 0; i < n; i++)
    {
        if (distance[i] < min_d && found[i] == false)
        {
            min_d = distance[i];
            min_i = i;
        }
    }
    return min_i;
}

Point2 RoadMap::getNearestEdgePoint(const Point2& p, ID& node1, ID& node2)
{
    std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
    Point2 nearest_p;
    node1 = 0;
    node2 = 0;
    for (auto from = getHeadNodeConst(); from != getTailNodeConst(); from++)
    {
        for (auto edge = getHeadEdgeConst(from); edge != getTailEdgeConst(from); edge++)
        {
            const RoadMap::Node* to = edge->to;
            if (to == nullptr) continue;
            auto dist2 = calcDist2FromLineSeg(from->data, to->data, p);
            if (dist2.first < min_dist2.first)
            {
                min_dist2 = dist2;
                node1 = from->data.id;
                node2 = to->data.id;
                nearest_p = dist2.second;
            }
        }
    }
    return nearest_p;
}

std::pair<double, Point2> RoadMap::calcDist2FromLineSeg(const Point2& from, const Point2& to, const Point2& p)
{
    // Ref. https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    Point2 delta = to - from;
    double l2 = delta.x * delta.x + delta.y * delta.y;
    if (l2 < DBL_EPSILON)
    {
        Point2 dp = p - from;
        return std::make_pair(dp.x * dp.x + dp.y * dp.y, from);
    }
    double t = std::max(0., std::min(1., (p - from).dot(to - from) / l2));
    Point2 projection = from + t * (to - from);
    Point2 dp = p - projection;
    double dist2 = dp.x * dp.x + dp.y * dp.y;
    return std::make_pair(dist2, projection);
}

void RoadMap::removeRoutingVariables()
{
    m_dest_node = nullptr;
    m_distance.clear();
    m_found.clear();
    m_next_idx.clear();
    m_indexed_node_lookup.clear();
}


} // End of 'dg'
