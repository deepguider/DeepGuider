#include "map.hpp"
#include "utils/opencx.hpp"
#include <locale>
#include <codecvt>
#include <wchar.h>

#define MAP_BUF_SIZE               (1024)

namespace dg
{

bool Map::load(const char* filename, bool load_from_latlon)
{
    removeAll();

    FILE* fid = fopen(filename, "rt+,ccs=UTF-8");
    if (fid == nullptr) return false;

    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

    wchar_t buffer[MAP_BUF_SIZE];
    wchar_t* context;
    while (!feof(fid))
    {
        if (fgetws(buffer, MAP_BUF_SIZE, fid) == nullptr) break;
        wchar_t* token;
        if ((token = wcstok(buffer, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
        if (token[0] == 'N' || token[0] == 'n')
        {
            // Read nodes
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v1 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v2 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            int type = std::stoi(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            int floor = std::stoi(cx::trimBoth(token), nullptr, 10);

            if(load_from_latlon) addNode(Node(id, toMetric(LatLon(v1, v2)), type, floor));
            else addNode(Node(id, v1, v2, type, floor));
        }
        else if (token[0] == 'E' || token[0] == 'e')
        {
            // Read edges
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            int type = std::stoi(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id1 = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id2 = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double length = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            bool directed = (std::stoi(cx::trimBoth(token), nullptr, 10) != 0);

            addEdge(Edge(id, type, id1, id2, length, directed));
        }
        else if (token[0] == 'P' || token[0] == 'p')
        {
            // Read pois
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v1 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v2 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            std::wstring name_token = token;
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            std::wstring floor_token = token;
            while ((token = wcstok(nullptr, L",", &context)) != nullptr)
            {
                name_token = name_token + L"," + floor_token;
                floor_token = token;
            }
            std::wstring name = cx::trimBoth(name_token);
            int floor = std::stoi(cx::trimBoth(floor_token), nullptr, 10);

            if(load_from_latlon) addPOI(POI(id, toMetric(LatLon(v1, v2)), name, floor));
            else addPOI(POI(id, v1, v2, name, floor));
        }
        else if (token[0] == 'V' || token[0] == 'v')
        {
            // Read views
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            ID id = std::stoll(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v1 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double v2 = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            int floor = std::stoi(cx::trimBoth(token), nullptr, 10);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            double heading = std::stod(cx::trimBoth(token), nullptr);
            if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto MAP_LOADMAP_FAIL;
            std::string date = converter.to_bytes(cx::trimBoth(token));

            if (load_from_latlon) addView(StreetView(id, toMetric(LatLon(v1, v2)), floor, heading, date));
            else addView(StreetView(id, v1, v2, floor, heading, date));
        }
    }
    fclose(fid);
    return true;

MAP_LOADMAP_FAIL:
    removeAll();
    fclose(fid);
    return false;
}

bool Map::save(const char* filename, bool save_as_latlon) const
{
    if (isEmpty(false)) return false;

    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

    FILE* file = fopen(filename, "wt");
    if (file == nullptr) return false;
    if (save_as_latlon)
    {
        fprintf(file, "# NODE, ID, latitude [deg], longitude [deg], type, floor\n");
        fprintf(file, "# EDGE, ID, type, ID(from_node), ID(to_node), length, directed\n");
        fprintf(file, "# POI, ID, latitude [deg], longitude [deg], name, floor\n");
        fprintf(file, "# VIEW, ID, latitude [deg], longitude [deg], floor, heading [rad], date [yyyy-mm-dd]\n");

        // Write nodes
        for (auto itr_node = nodes.begin(); itr_node != nodes.end(); itr_node++)
        {
            LatLon ll = toLatLon(*itr_node);
            fprintf(file, "NODE, %zd, %lf, %lf, %d, %d\n", itr_node->id, ll.lat, ll.lon, itr_node->type, itr_node->floor);
        }

        // Write edges
        for (auto itr_edge = edges.begin(); itr_edge != edges.end(); itr_edge++)
        {
            fprintf(file, "EDGE, %zd, %d, %zd, %zd, %lf, %d\n", itr_edge->id, itr_edge->type, itr_edge->node_id1, itr_edge->node_id2, itr_edge->length, (int)itr_edge->directed);
        }

        // Write pois
        for (auto itr_poi = pois.begin(); itr_poi != pois.end(); itr_poi++)
        {
            LatLon ll = toLatLon(*itr_poi);
            std::string name = converter.to_bytes(itr_poi->name);
            fprintf(file, "POI, %zd, %lf, %lf, %s, %d\n", itr_poi->id, ll.lat, ll.lon, name.c_str(), itr_poi->floor);
        }

        // Write views
        for (auto itr_view = views.begin(); itr_view != views.end(); itr_view++)
        {
            LatLon ll = toLatLon(*itr_view);
            fprintf(file, "VIEW, %zd, %lf, %lf, %d, %lf, %s\n", itr_view->id, ll.lat, ll.lon, itr_view->floor, itr_view->heading, itr_view->date.c_str());
        }
    }
    else
    {
        fprintf(file, "# NODE, ID, X [m], Y [m], type, floor\n");
        fprintf(file, "# EDGE, ID, type, ID(from_node), ID(to_node), length, directed\n");
        fprintf(file, "# POI, ID, X [m], Y [m], name, floor\n");
        fprintf(file, "# VIEW, ID, X [m], Y [m], floor, heading [rad], date [yyyy-mm-dd]\n");

        // Write nodes
        for (auto itr_node = nodes.begin(); itr_node != nodes.end(); itr_node++)
        {
            fprintf(file, "NODE, %zd, %lf, %lf, %d, %d\n", itr_node->id, itr_node->x, itr_node->y, itr_node->type, itr_node->floor);
        }

        // Write edges
        for (auto itr_edge = edges.begin(); itr_edge != edges.end(); itr_edge++)
        {
            fprintf(file, "EDGE, %zd, %d, %zd, %zd, %lf, %d\n", itr_edge->id, itr_edge->type, itr_edge->node_id1, itr_edge->node_id2, itr_edge->length, (int)itr_edge->directed);
        }

        // Write pois
        for (auto itr_poi = pois.begin(); itr_poi != pois.end(); itr_poi++)
        {
            std::string name = converter.to_bytes(itr_poi->name);
            fprintf(file, "POI, %zd, %lf, %lf, %s, %d\n", itr_poi->id, itr_poi->x, itr_poi->y, name.c_str(), itr_poi->floor);
        }

        // Write views
        for (auto itr_view = views.begin(); itr_view != views.end(); itr_view++)
        {
            fprintf(file, "VIEW, %zd, %lf, %lf, %d, %lf, %s\n", itr_view->id, itr_view->x, itr_view->y, itr_view->floor, itr_view->heading, itr_view->date.c_str());
        }
    }
    fclose(file);
    return true;
}

bool Map::copyTo(Map* dest) const
{
    if (dest == nullptr) return false;

    dest->removeAll();
    dest->setReference(getReference());
    for (auto node = nodes.begin(); node != nodes.end(); node++)
        dest->addNode(*node);
    for (auto edge = edges.begin(); edge != edges.end(); edge++)
        dest->addEdge(*edge);
    for (auto poi = pois.begin(); poi != pois.end(); poi++)
        dest->addPOI(*poi);
    for (auto view = views.begin(); view != views.end(); view++)
        dest->addView(*view);
    return true;
}

bool Map::getPath(Point2 p1, Point2 p2, Path& path)
{
    // reset path
    path.pts.clear();

    // find nearest edge
    Point2 p1_ep, p2_ep;
    const Edge* edge1 = getNearestEdge(p1, p1_ep);
    const Edge* edge2 = getNearestEdge(p2, p2_ep);
    if (edge1 == nullptr || edge2 == nullptr) return false;

    // when p1 and p2 are projected on the same edge
    if (edge1 == edge2 || edge1->node_id1 == edge2->node_id1 && edge1->node_id2 == edge2->node_id2 || edge1->node_id1 == edge2->node_id2 && edge1->node_id2 == edge2->node_id1)
    {
        path.pts.push_back(PathNode(p1));
        path.pts.push_back(PathNode(p2));
        return true;
    }

    // start node
    const Node* from = getNode(edge1->node_id1);
    if (from == nullptr) return false;
    double d1 = norm(p1_ep - *from);
    if (d1 > edge1->length / 2)
    {
        from = getNode(edge1->node_id2);
        d1 = norm(p1_ep - *from);
    }

    // dest node
    const Node* to = getNode(edge2->node_id1);
    if (to == nullptr) return false;
    double d2 = norm(p2_ep - *to);
    if (d2 > edge2->length / 2)
    {
        to = getNode(edge2->node_id2);
        d2 = norm(p2_ep - *to);
    }

    // index of start & dest node
    int from_idx = getNodeIndex(from->id);
    int to_idx = getNodeIndex(to->id);
    if (from_idx < 0 || to_idx < 0) return false;

    // initlize temporal variables
    initializeRouterVariables(to);

    // bread-first search
    int nNodes = (int)countNodes();
    while (!m_found[from_idx])
    {
        int best_ni = chooseBestUnvisited(m_distance, m_found);
        if (best_ni < 0) break;
        m_found[best_ni] = true;
        const Node& best = nodes[best_ni];

        for (auto eid = best.edge_ids.begin(); eid != best.edge_ids.end(); eid++)
        {
            Node* node = getConnectedNode(&best, *eid);
            if (node == nullptr) continue;
            int j = getNodeIndex(node->id);
            if (m_found[j] == false)
            {
                const Edge* edge = getEdge(*eid);
                if (edge && (m_distance[best_ni] + edge->length < m_distance[j]))
                {
                    m_distance[j] = m_distance[best_ni] + edge->length;
                    m_next_idx[j] = best_ni;
                }
            }
        }
    }
    if (!m_found[from_idx]) return false;

    // build path
    for (auto idx = from_idx; idx != -1; idx = m_next_idx[idx])
    {
        ID edge_id = 0;
        if (m_next_idx[idx] >= 0)
        {
            const Edge* edge = getEdge(nodes[idx].id, nodes[m_next_idx[idx]].id);
            if (edge) edge_id = edge->id;
        }
        path.pts.push_back(PathNode(nodes[idx], edge_id));
        if (idx == to_idx) break;
    }
    if (path.pts.empty() || path.pts.front().node_id != from->id || path.pts.back().node_id != to->id) return false;

    // check & remove abnormal turn-around path (in case projected edge point is not node point)
    if ((int)path.pts.size() > 1 && d1 > DBL_EPSILON)
    {
        if (path.pts[1].node_id == edge1->node_id1 || path.pts[1].node_id == edge1->node_id2) path.pts.erase(path.pts.begin());
    }
    if ((int)path.pts.size() > 1 && d2 > DBL_EPSILON)
    {
        int npath = (int)path.pts.size();
        if (path.pts[npath - 2].node_id == edge2->node_id1 || path.pts[npath - 2].node_id == edge2->node_id2) path.pts.pop_back();
    }

    // add start_pos and dest_pos in path (in case they are not node point)
    double nd1 = norm(p1 - *from);
    double nd2 = norm(p2 - *to);
    if (nd1 > DBL_EPSILON) path.pts.insert(path.pts.begin(), PathNode(p1));
    if (nd2 > DBL_EPSILON) path.pts.push_back(PathNode(p2));

    return true;
}

bool Map::getPath(ID from, ID to, Path& path)
{
    Node* n1 = getNode(from);
    Node* n2 = getNode(to);
    if (n1 == nullptr || n2 == nullptr) return false;
    return getPath(*n1, *n2, path);
}

Node* Map::getNearestNode(const Point2& p)
{
    Node* node = nullptr;
    double min_d2 = DBL_MAX;
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 < min_d2)
        {
            min_d2 = d2;
            node = &(*it);
        }
    }
    return node;
}

const Node* Map::getNearestNode(const Point2& p) const
{
    const Node* node = nullptr;
    double min_d2 = DBL_MAX;
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 < min_d2)
        {
            min_d2 = d2;
            node = &(*it);
        }
    }
    return node;
}


std::vector<Node*> Map::getNearNodes(const Point2& p, double search_radius)
{
    std::vector<Node*> results;
    double radius2 = search_radius * search_radius;
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 <= radius2)
        {
            results.push_back(&(*it));
        }
    }
    return results;
}

std::vector<const Node*> Map::getNearNodes(const Point2& p, double search_radius) const
{
    std::vector<const Node*> results;
    double radius2 = search_radius * search_radius;
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 <= radius2)
        {
            results.push_back(&(*it));
        }
    }
    return results;
}

std::vector<const Node*> Map::getConnectedNearNodes(const Node* node, double search_radius) const
{
    std::vector<const Node*> results;
    if (node == nullptr) return results;

    std::vector<double> distance;

    std::list<const Node*> open;
    open.push_back(node);
    results.push_back(node);

    std::map<ID, int> lookup_tmp;
    int idx = 0;
    lookup_tmp.insert(std::make_pair(node->id, idx++));

    while (!open.empty())
    {
        // pick next
        auto from = open.front();
        open.pop_front();

        for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
        {
            const Node* to = getConnectedNode(from, *it);
            if (to == nullptr) continue;

            // check radius
            double d = norm(*to - *node);
            if (d > search_radius) continue;

            // check duplication
            auto found = lookup_tmp.find(to->id);
            if (found == lookup_tmp.end()) continue;

            open.push_back(to);
            results.push_back(to);
            lookup_tmp.insert(std::make_pair(to->id, idx++));
        }
    }
    return results;
}

std::vector<Node*> Map::getConnectedNearNodes(Node* node, double search_radius)
{
    std::vector<Node*> results;
    if (node == nullptr) return results;

    std::vector<double> distance;

    std::list<Node*> open;
    open.push_back(node);
    results.push_back(node);

    std::map<ID, int> lookup_tmp;
    int idx = 0;
    lookup_tmp.insert(std::make_pair(node->id, idx++));

    while (!open.empty())
    {
        // pick next
        auto from = open.front();
        open.pop_front();

        for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
        {
            Node* to = getConnectedNode(from, *it);
            if (to == nullptr) continue;

            // check radius
            double d = norm(*to - *node);
            if (d > search_radius) continue;

            // check duplication
            auto found = lookup_tmp.find(to->id);
            if (found == lookup_tmp.end()) continue;

            open.push_back(to);
            results.push_back(to);
            lookup_tmp.insert(std::make_pair(to->id, idx++));
        }
    }
    return results;
}

Edge* Map::getNearestEdge(const Point2& p, Point2& nearest_edge_point)
{
    std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
    Edge* min_edge = nullptr;
    for (auto edge = edges.begin(); edge != edges.end(); edge++)
    {
        Node* node1 = getNode(edge->node_id1);
        Node* node2 = getNode(edge->node_id2);
        if (node1 == nullptr || node2 == nullptr) continue;

        auto dist2 = calcDist2FromLineSeg(*node1, *node2, p);
        if (dist2.first < min_dist2.first)
        {
            min_dist2 = dist2;
            min_edge = &(*edge);
        }
    }
    nearest_edge_point = min_dist2.second;

    return min_edge;
}

const Edge* Map::getNearestEdge(const Point2& p, Point2& nearest_edge_point) const
{
    std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
    const Edge* min_edge = nullptr;
    for (auto edge = edges.begin(); edge != edges.end(); edge++)
    {
        const Node* node1 = getNode(edge->node_id1);
        const Node* node2 = getNode(edge->node_id2);
        if (node1 == nullptr || node2 == nullptr) continue;

        auto dist2 = calcDist2FromLineSeg(*node1, *node2, p);
        if (dist2.first < min_dist2.first)
        {
            min_dist2 = dist2;
            min_edge = &(*edge);
        }
    }
    nearest_edge_point = min_dist2.second;

    return min_edge;
}

Pose2 Map::getNearestMapPose(const Pose2& pose_m) const
{
    // Find the nearest edge
    Point2 ep;
    const Edge* edge = getNearestEdge(pose_m, ep);
    if (edge == nullptr) return pose_m;

    // determine edge direction
    const Node* node1 = getNode(edge->node_id1);
    const Node* node2 = getNode(edge->node_id2);
    if (node1 == nullptr || node2 == nullptr) return pose_m;
    Point2 v1 = *node2 - *node1;
    Point2 v2 = *node1 - *node2;
    double dtheta1 = fabs(cx::trimRad(pose_m.theta - atan2(v1.y, v1.x)));
    double dtheta2 = fabs(cx::trimRad(pose_m.theta - atan2(v2.y, v2.x)));
    double theta = (dtheta1 <= dtheta2) ? atan2(v1.y, v1.x) : atan2(v2.y, v2.x);

    Pose2 pose = ep;
    pose.theta = theta;
    return pose;
}

Pose2 Map::getNearestPathPose(const Path& path, const Pose2& pose_m)
{
    if (path.empty()) return pose_m;

    // Find the nearest path point
    int min_path_idx = -1;
    std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
    for (int i = 0; i < (int)path.pts.size() - 1; i++)
    {
        auto dist2 = calcDist2FromLineSeg(path.pts[i], path.pts[i + 1], pose_m);
        if (dist2.first < min_dist2.first)
        {
            min_dist2 = dist2;
            min_path_idx = i;
        }
    }
    if (min_path_idx <= 0) return pose_m;

    // determine path direction
    Point2 v1 = path.pts[min_path_idx + 1] - path.pts[min_path_idx];
    Point2 v2 = path.pts[min_path_idx] - path.pts[min_path_idx + 1];
    double dtheta1 = fabs(cx::trimRad(pose_m.theta - atan2(v1.y, v1.x)));
    double dtheta2 = fabs(cx::trimRad(pose_m.theta - atan2(v2.y, v2.x)));
    double theta = (dtheta1 <= dtheta2) ? atan2(v1.y, v1.x) : atan2(v2.y, v2.x);

    Pose2 pose = min_dist2.second;
    pose.theta = theta;
    return pose;
}

int compare_poi(const void* a, const void* b)
{
    double x = ((std::pair<POI*, double>*)a)->second;
    double y = ((std::pair<POI*, double>*)b)->second;

    if (x > y)
        return 1;
    else if (x < y)
        return -1;
    return 0;
}

std::vector<POI*> Map::getPOI(std::wstring name, const Point2& p, double search_radius, bool sorted)
{
    std::vector<POI*> name_pois = getPOI(name);
    if (name_pois.empty()) return name_pois;

    if (!sorted)
    {
        std::vector<POI*> results;
        double radius2 = search_radius * search_radius;
        for (auto it = name_pois.begin(); it != name_pois.end(); it++)
        {
            double d2 = (p.x - (*it)->x) * (p.x - (*it)->x) + (p.y - (*it)->y) * (p.y - (*it)->y);
            if (d2 <= radius2)
            {
                results.push_back(&(*(*it)));
            }
        }
        return results;
    }

    // qsort in ascending order
    std::vector<std::pair<POI*, double>> arr;
    double radius2 = search_radius * search_radius;
    for (auto it = name_pois.begin(); it != name_pois.end(); it++)
    {
        double d2 = (p.x - (*it)->x) * (p.x - (*it)->x) + (p.y - (*it)->y) * (p.y - (*it)->y);
        if (d2 <= radius2)
        {
            arr.push_back(std::make_pair(&(*(*it)), d2));
        }
    }
    qsort(&(arr[0]), arr.size(), sizeof(arr[0]), compare_poi);

    std::vector<POI*> results;
    results.resize(arr.size());
    for (auto i = 0; i < arr.size(); i++)
    {
        results[i] = arr[i].first;
    }
    return results;
}

std::vector<POI*> Map::getNearPOIs(const Point2& p, double search_radius, bool sorted)
{
    if (!sorted)
    {
        std::vector<POI*> results;
        double radius2 = search_radius * search_radius;
        for (auto it = pois.begin(); it != pois.end(); it++)
        {
            double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
            if (d2 <= radius2)
            {
                results.push_back(&(*it));
            }
        }
        return results;
    }

    // qsort in ascending order
    std::vector<std::pair<POI*, double>> arr;
    double radius2 = search_radius * search_radius;
    for (auto it = pois.begin(); it != pois.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 <= radius2)
        {
            arr.push_back(std::make_pair(&(*it), d2));
        }
    }
    qsort(&(arr[0]), arr.size(), sizeof(arr[0]), compare_poi);

    std::vector<POI*> results;
    results.resize(arr.size());
    for (auto i = 0; i < arr.size(); i++)
    {
        results[i] = arr[i].first;
    }
    return results;
}

std::vector<std::wstring> Map::getNearPOINames(const Point2& p, double search_radius)
{
    std::vector<std::wstring> results;
    double radius2 = search_radius * search_radius;
    for (auto it = pois.begin(); it != pois.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 <= radius2)
        {
            results.push_back(it->name);
        }
    }
    return results;
}

std::vector<StreetView*> Map::getNearViews(const Point2& p, double search_radius)
{
    std::vector<StreetView*> results;
    double radius2 = search_radius * search_radius;
    for (auto it = views.begin(); it != views.end(); it++)
    {
        double d2 = (p.x - it->x) * (p.x - it->x) + (p.y - it->y) * (p.y - it->y);
        if (d2 <= radius2)
        {
            results.push_back(&(*it));
        }
    }
    return results;
}

bool Map::initializeRouterVariables(const Node* dest_node)
{
    if (!m_router_valid) resetRouterVariables();
    if (!dest_node) return false;
    if (dest_node == m_dest_node) return true;

    int nNodes = (int)countNodes();
    m_distance.resize(nNodes);
    m_found.resize(nNodes);
    m_next_idx.resize(nNodes);

    int dest_node_idx = getNodeIndex(dest_node->id);
    int i = 0;
    for (auto node = nodes.begin(); node != nodes.end(); i++, node++)
    {
        const Edge* edge = getEdge(node->id, dest_node->id);
        m_distance[i] = (edge) ? edge->length : DBL_MAX;
        m_found[i] = false;
        m_next_idx[i] = (edge) ? dest_node_idx : -1;
    }

    m_distance[dest_node_idx] = 0;
    m_found[dest_node_idx] = true;
    m_next_idx[dest_node_idx] = -1;

    m_dest_node = dest_node;
    m_router_valid = true;

    return true;
}

void Map::resetRouterVariables()
{
    m_distance.clear();
    m_found.clear();
    m_next_idx.clear();
    m_dest_node = nullptr;
    m_router_valid = false;
}

int Map::chooseBestUnvisited(const std::vector<double>& distance, const std::vector<bool>& found)
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

std::pair<double, Point2> calcDist2FromLineSeg(const Point2& from, const Point2& to, const Pose2& p, double turn_weight)
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
    if (turn_weight > 0)
    {
        double dh = cx::trimRad(p.theta - atan2(delta.y, delta.x));
        dist2 += turn_weight * dh * dh;
    }
    return std::make_pair(dist2, projection);
}


} // End of 'dg'
