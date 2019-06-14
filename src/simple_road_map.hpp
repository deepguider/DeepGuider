#ifndef __SIMPLE_ROAD_MAP__
#define __SIMPLE_ROAD_MAP__

#include "opencvx.hpp"

namespace dg
{

/** A 2D point(vector) */
typedef cv::Point2d Point;

/**
 * @brief <b>2D point(vector) with ID</b>
 *
 * A 2D point(vector) is defined with the identifier (ID; integer type).
 */
class PointID : public Point
{
public:
    /**
     * A constructor with ID assignment
     * @param _id The givne ID
     */
    PointID(int _id = -1) : id(_id) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param _x The given X
     * @param _y The given Y
     */
    PointID(int _id, double _x, double _y) : id(_id), cv::Point2d(_x, _y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The givne ID
     * @param p The given 2D point
     */
    PointID(int _id, cv::Point2d p) : id(_id), cv::Point2d(p) { }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const PointID& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const PointID& rhs) const { return (id != rhs.id); }

    /** The given identifider */
    int id;
};

} // End of 'dg'

#endif // End of '__SIMPLE_ROAD_MAP__'
