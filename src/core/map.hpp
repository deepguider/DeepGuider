#ifndef __MAP__
#define __MAP__

#include "core/basic_type.hpp"
#include "core/directed_graph.hpp"

namespace dg
{

/**
 * @brief 2D point on the earth with ID
 *
 * A 2D point in the geodesic notation is defined with the identifier (shortly ID).
 */
class LonLatID : public Point2ID
{
public:
    /**
     * A constructor with ID assignment
     * @param _id The givne ID
     */
    LonLatID(ID _id = 0) : Point2ID(_id), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param _x The given X
     * @param _y The given Y
     */
    LonLatID(ID _id, double _lon, double _lat) : Point2ID(_id, _lon, _lat), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param p The given 2D point
     */
    LonLatID(ID _id, const Point2& p) : Point2ID(_id, p), lon(x), lat(y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param pid The given 2D point with ID
     */
    LonLatID(const Point2ID& pid) : Point2ID(pid), lon(x), lat(y) { }

    /** Latitude */
    double& lon;

    /** Latitude */
    double& lat;
};

class NodeInfo : public LonLatID
{
public:
    NodeInfo(ID _id = 0, double _lon = 0, double _lat = 0, int _type = 0, int _floor = 0) : LonLatID(_id, _lon, _lat), type(_type), floor(_floor) { }

    NodeInfo(ID _id, const LonLat& p, int _type = 0, int _floor = 0) : LonLatID(_id, p), type(_type), floor(_floor) { }

    NodeInfo(const LonLatID& p, int _type = 0, int _floor = 0) : LonLatID(p), type(_type), floor(_floor) { }

    int type;

    int floor;

    std::vector<ID> sv_ids;

    std::vector<std::string> pois;
};

class EdgeInfo
{
public:
    EdgeInfo(double _width = 1, double _length = 1, int _type = 0) : width(_width), length(_length), type(_type) { }

    int type;

    double width;

    double length;
};

typedef dg::DirectedGraph<NodeInfo, EdgeInfo> Map;

} // End of 'dg'

#endif // End of '__MAP__'
