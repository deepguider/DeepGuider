#ifndef __UTM_CONVERTER__
#define __UTM_CONVERTER__

#include "core/basic_type.hpp"

#ifndef UTM_H
extern int  LatLonToUTMXY(double lat, double lon, int zone, double& x, double& y);
extern void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double& lat, double& lon);
#endif

namespace dg
{

/**
 * @brief 2D point in UTM notation
 *
 * A 2D point is represented with more information (the zone number and a flag in the southern hemisphere or not)
 * The Republic of Korea is roughly located at the zone number 52 in the northern hemisphere.
 *
 * @see Universal Transverse Mercator coordinate system, Wikipedia: https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
 */
class Point2UTM : public Point2
{
public:
    /**
     * A constructor with assignment
     * @param _x The given X position (Unit: [m])
     * @param _y The given Y position (Unit: [m])
     * @param _zone The given zone number (default = 52, utm zone of korea)
     * @param _is_south A flag whether this point is at the southern hemisphere.
     */
    Point2UTM(double _x = 0, double _y = 0, int _zone = 52, bool _is_south = false) : Point2(_x, _y), zone(_zone), is_south(_is_south) { }

    /** The zone number */
    int zone;

    /** A flag in the southern hemisphere or not */
    bool is_south;
};


/**
 * @brief A converter between geodesic and UTM notations
 *
 * This converter can change geodesic position to its metric position in UTM notation and vice versa.
 * It provides static functions such as cvtLatLon2UTM and cvtUTM2LatLon.
 * It is also possible to assign a reference point (as like the origin) with setReference function.
 * Two member functions, toMetric and toLatLon, are based on the reference point.
 */
class UTMConverter
{
public:
    /**
     * Convert geodesic position to metric position based on the reference point
     * @param ll The given geodesic position
     * @return The converted metric position
     */
    Point2 toMetric(const LatLon& ll) const
    {
        Point2UTM utm = cvtLatLon2UTM(ll);
        if (m_refer_utm.zone == utm.zone && m_refer_utm.is_south == utm.is_south) return utm - m_refer_utm;
        // TODO: How to calculate when two zones are different
        return utm;
    }

    /**
     * Convert metric position to geodesic position based on the reference point
     * @param metric The given metric position
     * @return The converted geodesic position
     */
    LatLon toLatLon(const Point2& metric) const
    {
        // TODO: How to calculate when the given metric is beyond of the reference zone
        Point2UTM utm = m_refer_utm;
        utm.x += metric.x;
        utm.y += metric.y;
        return cvtUTM2LatLon(utm);
    }

    /**
     * Assign the reference point in UTM notation
     * @param utm The reference in UTM notation
     * @return True if successful (false if failed)
     */
    bool setReference(const Point2UTM& utm) { m_refer_utm = utm; return true; }

    /**
     * Assign the reference point in geodesic notation
     * @param ll The reference in geodesic notation
     * @return True if successful (false if failed)
     */
    bool setReference(const LatLon& ll) { return setReference(cvtLatLon2UTM(ll)); }

    /**
     * Get the reference point
     * @return The reference point
     */
    Point2UTM getReference() const { return m_refer_utm; }

    /**
     * Convert geodesic position to UTM position
     * @param ll The given geodesic position
     * @return The converted UTM position
     */
    static Point2UTM cvtLatLon2UTM(const LatLon& ll)
    {
        Point2UTM utm;
        utm.zone = LatLonToUTMXY(ll.lat, ll.lon, -1, utm.x, utm.y);
        return utm;
    }

    /**
     * Convert UTM position to geodesic position
     * @param utm The given UTM position
     * @return The converted geodesic position
     */
    static LatLon cvtUTM2LatLon(const Point2UTM& utm)
    {
        LatLon ll;
        UTMXYToLatLon(utm.x, utm.y, utm.zone, utm.is_south, ll.lat, ll.lon);
        ll.lat *= 180 / CV_PI; // [rad] to [deg]
        ll.lon *= 180 / CV_PI; // [rad] to [deg]
        return ll;
    }

protected:
    /** The reference point in UTM notation */
    Point2UTM m_refer_utm;
};

} // End of 'dg'

#endif // End of '__UTM_CONVERTER__'
