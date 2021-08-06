#include "utm_converter.hpp"

#ifndef UTM_H
    extern int  LatLonToUTMXY(double lat, double lon, int zone, double& x, double& y);
    extern void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double& lat, double& lon);
#endif

namespace dg
{

Point2 UTMConverter::toMetric(const LatLon& ll) const
{
    Point2UTM utm = cvtLatLon2UTM(ll);
    if (m_refer_utm.zone == utm.zone && m_refer_utm.is_south == utm.is_south) return utm - m_refer_utm;
    // TODO: How to calculate when two zones are different
    return utm;
}

LatLon UTMConverter::toLatLon(const Point2& metric) const
{
    // TODO: How to calculate when the given metric is beyond of the reference zone
    Point2UTM utm = m_refer_utm;
    utm.x += metric.x;
    utm.y += metric.y;
    return cvtUTM2LatLon(utm);
}

Point2UTM UTMConverter::cvtLatLon2UTM(const LatLon& ll)
{
    Point2UTM utm;
    utm.zone = LatLonToUTMXY(ll.lat, ll.lon, -1, utm.x, utm.y);
    return utm;
}

LatLon UTMConverter::cvtUTM2LatLon(const Point2UTM& utm)
{
    LatLon ll;
    UTMXYToLatLon(utm.x, utm.y, utm.zone, utm.is_south, ll.lat, ll.lon);
    ll.lat *= 180 / CV_PI; // [rad] to [deg]
    ll.lon *= 180 / CV_PI; // [rad] to [deg]
    return ll;
}

} // End of 'dg'
