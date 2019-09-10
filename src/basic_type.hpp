#ifndef __DATA_STRUCTURE__
#define __DATA_STRUCTURE__

#include "opencx.hpp"

namespace dg
{

/** Time stamp (unit: [sec]) */
typedef double Timestamp;

/**
 * @brief 2D point
 *
 * A 2D point is defined in the rectangular coordinate.
 * Its member variables includes x and y.
 *
 * @see Polar2 2D vector in the polar coordinate
 */
typedef cv::Point2d Point2;

/**
 * @brief 2D vector in the polar coordinate
 * 
 * A 2D vector is defined in the polar coordinate.
 * Its member variables includes linear component (lin) and angular component (ang).
 *
 * @see Point2 2D point in the rectangular coordinate
 */
class Polar2
{
public:
    // The default constructor
    Polar2() : lin(0), ang(0) { }

    Polar2(double _lin, double _ang) : lin(_lin), ang(_ang) { }

    bool operator==(const Polar2& rhs) const { return (lin == rhs.lin) && (ang == rhs.ang); }

    bool operator!=(const Polar2& rhs) const { return (lin != rhs.lin) || (ang != rhs.ang); }

    double lin;

    double ang;
};

/**
 * @brief 2D pose
 *
 * 2D Pose is represented in the rectangular coordinate.
 * Its member variables includes 2D position (x, y) and orientation (theta).
 */
class Pose2 : public Point2
{
public:
    // The default constructor
    Pose2() : theta(0) { }

    Pose2(double _x, double _y, double _t = 0) : Point2(_x, _y), theta(_t) { }

    Pose2(const Point2& pt, double _t = 0) : Point2(pt), theta(_t) { }

    Pose2(const Pose2& pose) : Point2(pose.x, pose.y), theta(pose.theta) { }

    bool operator==(const Pose2& rhs) const { return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta); }

    bool operator!=(const Pose2& rhs) const { return (x != rhs.x) || (y != rhs.y) || (theta != rhs.theta); }

    double theta;
};

/**
 * @brief 2D point in the geodesic notation
 *
 * A 2D point is represented in the geodesic notation.
 * Its member variables includes lon (longitude) and lat (latitude).
 * Similarly to Point2, two member variables can be also referred as x and y, respectively.
 *
 * @see Point2 2D vector in the rectangular coordinate
 */
class LatLon : public Point2
{
public:
    /**
     * The default constructor
     */
    LatLon() : lon(x), lat(y) { }

    /**
     * A constructor with initialization
     * @param _lon A value for longitude
     * @param _lat A value for latitude
     */
    LatLon(double _lon, double _lat) : Point2(_lon, _lat), lon(x), lat(y) { }

    /**
     * A constructor with initialization
     * @param p A 2D point
     */
    LatLon(const Point2& p) : Point2(p), lon(x), lat(y) { }

    /**
     * Overriding the assignment operator
     * @param p A 2D point in the right-hand side
     * @return The assigned instance
     */
    LatLon& operator=(const Point2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /** Latitude */
    double& lat;

    /** Latitude */
    double& lon;
};

} // End of 'dg'

#endif // End of '__DATA_STRUCTURE__'
