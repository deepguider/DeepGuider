#ifndef __BASIC_TYPE__
#define __BASIC_TYPE__

#include "opencv2/opencv.hpp"

namespace dg
{

/** Time stamp (unit: [sec]) */
typedef double Timestamp;

/**
 * Identifier<br>
 * When the value of ID is given as 0, it means unknown (invalid) ID.
 */
typedef uint64_t ID;

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
    /**
     * The default constructor
     */
    Polar2() : lin(0), ang(0) { }

    /**
     * A constructor with initialization
     * @param _lin A value for linear component (Unit: [m])
     * @param _ang A value for angular component (Unit: [rad])
     */
    Polar2(double _lin, double _ang) : lin(_lin), ang(_ang) { }

    /**
     * Overriding the equality operator
     * @param rhs A 2D vector in the right-hand side
     * @return The assigned instance
     */
    bool operator==(const Polar2& rhs) const { return (lin == rhs.lin) && (ang == rhs.ang); }

    /**
     * Overriding the inequality operator
     * @param rhs A 2D vector in the right-hand side
     * @return The assigned instance
     */
    bool operator!=(const Polar2& rhs) const { return (lin != rhs.lin) || (ang != rhs.ang); }

    /** The linear component (Unit: [m]) */
    double lin;

    /** The angular component (Unit: [rad]) */
    double ang;
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
class LonLat : public Point2
{
public:
    /**
     * The default constructor
     */
    LonLat() : lon(x), lat(y) { }

    /**
     * A constructor with initialization
     * @param _lon A value for longitude
     * @param _lat A value for latitude
     */
    LonLat(double _lon, double _lat) : Point2(_lon, _lat), lon(x), lat(y) { }

    /**
     * A constructor with initialization
     * @param p A 2D point
     */
    LonLat(const Point2& p) : Point2(p), lon(x), lat(y) { }

    /**
     * Overriding the assignment operator
     * @param rhs A 2D point in the right-hand side
     * @return The assigned instance
     */
    LonLat& operator=(const Point2& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    /** Latitude */
    double& lon;

    /** Latitude */
    double& lat;
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
    /**
     * The default constructor
     */
    Pose2() : theta(0) { }

    /**
     * A constructor with initialization
     * @param _x A value for x (Unit: [m])
     * @param _y A value for y (Unit: [m])
     * @param _t A value for t (Unit: [rad])
     */
    Pose2(double _x, double _y, double _t = 0) : Point2(_x, _y), theta(_t) { }

    /**
     * A constructor with initialization
     * @param pt A value for 2D point (Unit: [m])
     * @param _t A value for t (Unit: [rad])
     */
    Pose2(const Point2& pt, double _t = 0) : Point2(pt), theta(_t) { }

    /**
     * A constructor with initialization
     * @param pose A value for 2D pose (Unit: [m] and [rad])
     */
    Pose2(const Pose2& pose) : Point2(pose.x, pose.y), theta(pose.theta) { }

    /**
     * Overriding the equality operator
     * @param rhs A 2D pose in the right-hand side
     * @return The assigned instance
     */
    bool operator==(const Pose2& rhs) const { return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta); }

    /**
     * Overriding the inequality operator
     * @param rhs A 2D pose in the right-hand side
     * @return The assigned instance
     */
    bool operator!=(const Pose2& rhs) const { return (x != rhs.x) || (y != rhs.y) || (theta != rhs.theta); }

    /** Orientation (Unit: [rad]) */
    double theta;
};

/**
 * @brief 2D point with ID
 *
 * A 2D point is defined with the identifier (shortly ID).
 */
class Point2ID : public Point2
{
public:
    /**
     * A constructor with ID assignment
     * @param _id The given ID
     */
    Point2ID(ID _id = 0) : id(_id) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The given ID
     * @param _x The given X
     * @param _y The given Y
     */
    Point2ID(ID _id, double _x, double _y) : id(_id), Point2(_x, _y) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The given ID
     * @param p The given 2D point
     */
    Point2ID(ID _id, const Point2& p) : id(_id), Point2(p) { }

    /**
     * Check equality
     * @param rhs The right-hand side
     * @return Equality of two operands
     */
    bool operator==(const Point2ID& rhs) const { return (id == rhs.id); }

    /**
     * Check inequality
     * @param rhs The right-hand side
     * @return Inequality of two operands
     */
    bool operator!=(const Point2ID& rhs) const { return (id != rhs.id); }

    /** The given identifier */
    ID id;
};

} // End of 'dg'

#endif // End of '__BASIC_TYPE__'
