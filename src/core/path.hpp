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
     * @param _node_id ID of a node
     * @param _edge_id ID of an edge
     */
    PathElement(ID _node_id = 0, ID _edge_id = 0) : node_id(_node_id), edge_id(_edge_id) { }

    /** Node ID */
    ID node_id;

    /**
     * Edge ID<br>
     * The last element has zero ID.
     */
    ID edge_id;
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

    /** gps position of the starting point */
    LatLon start_pos;

    /** gps position of the destination point */
    LatLon dest_pos;

    /** A series of nodes and edges for a path */
    std::vector<PathElement> pts;

    /** Save data to file */
    bool save(const char* filename)
    {
        if (pts.empty()) return false;

        FILE* file = fopen(filename, "wt");
        if (file == nullptr) return false;

        for(size_t i=0; i<pts.size(); i++)
            fprintf(file, "%zd, %zd\n", pts[i].node_id, pts[i].edge_id);

        fclose(file);
        return true;
    }

    /** Load data from file */
    bool load(const char* filename)
    {
        FILE* fid = fopen(filename, "rt");
        if (fid == nullptr) return false;

        pts.clear();

        const int BUF_SIZE = 512;
        char buffer[BUF_SIZE];
        while (!feof(fid))
        {
            if (fgets(buffer, BUF_SIZE, fid) == nullptr) break;
            char* token;
            if ((token = strtok(buffer, ",")) == nullptr) break;
            ID nid = strtoll(token, nullptr, 10);
            if ((token = strtok(nullptr, ",")) == nullptr) break;
            ID eid = strtoll(token, nullptr, 10);

            pts.push_back(PathElement(nid, eid));
        }
        fclose(fid);
        return true;
    }

};

} // End of 'dg'

#endif // End of '__PATH__'
