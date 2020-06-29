#ifndef __BASIC_TYPE__
#define __BASIC_TYPE__

// Disable additional macro definition of 'min' and 'max' in 'windows.h'
#define NOMINMAX

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
 * @brief 2D point in the geodesic notation
 *
 * A 2D point is represented in the geodesic notation.
 * Its member variables includes lon (longitude) and lat (latitude).
 *
 * @see Point2 2D vector in the rectangular coordinate
 * @see Polar2 2D vector in the polar coordinate
 */
class LatLon
{
public:
    /**
     * The default constructor
     */
    LatLon() : lat(0), lon(0) { }

    /**
     * A constructor with initialization
     * @param _lat A value for latitude (Unit: [deg])
     * @param _lon A value for longitude (Unit: [deg])
     */
    LatLon(double _lat, double _lon) : lat(_lat), lon(_lon) { }

    /**
     * Overriding the equality operator
     * @param rhs A 2D vector in the right-hand side
     * @return The assigned instance
     */
    bool operator==(const LatLon& rhs) const { return (lat == rhs.lat) && (lon == rhs.lon); }

    /**
     * Overriding the inequality operator
     * @param rhs A 2D vector in the right-hand side
     * @return The assigned instance
     */
    bool operator!=(const LatLon& rhs) const { return (lat != rhs.lat) || (lon != rhs.lon); }

    /** Latitude (Unit: [deg]) */
    double lat;

    /** Longitude (Unit: [deg]) */
    double lon;
};

/**
 * @brief 2D point in the rectangular coordinate
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
 * @brief 2D pose
 *
 * 2D pose is represented in the rectangular coordinate.
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
    Point2ID(ID _id, double _x, double _y) : Point2(_x, _y), id(_id) { }

    /**
     * A constructor with ID, x, and y assignment
     * @param _id The given ID
     * @param p The given 2D point
     */
    Point2ID(ID _id, const Point2& p) : Point2(p), id(_id) { }

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

/**
 * @brief 2D pose on topological maps
 *
 * A robot's location is minimally represented with a node and its connecting edge on topological maps.
 * The node is defined as <i>the reference node</i>.
 * A member variable dist is the distance from the reference node, which presents more exact location of the robot on the edge.
 * Since the robot is not exactly on the edge, more information such as the heading angle and lateral offset is given for further usages.
 */
class TopometricPose
{
public:
    /**
     * A constructor with assignment
     * @param _node_id ID of the reference node
     * @param _edge_idx Index of the currently moving edge
     * @param _dist The traveled distance from the reference node (Unit: [m])
     * @param _head The heading angle with respect to the direction of the currently moving edge (Unit: [rad])
     */
    TopometricPose(ID _node_id = 0, int _edge_idx = 0, double _dist = 0, double _head = 0) : node_id(_node_id), edge_idx(_edge_idx), dist(_dist), head(_head) { }

    /** ID of the reference node (also defined as the previously departed node) */
    ID node_id;

    /** Index of the currently moving edge */
	int edge_idx;

    /** The traveled distance from the reference node (Unit: [m]) */
    double dist;

    /** The heading angle with respect to the direction of the currently moving edge (Unit: [rad]) */
    double head;
};

} // End of 'dg'

#endif // End of '__BASIC_TYPE__'
