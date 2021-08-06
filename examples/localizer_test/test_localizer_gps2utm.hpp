#ifndef __TEST_LOCALIZER_GPS2UTM__
#define __TEST_LOCALIZER_GPS2UTM__

#include "dg_core.hpp"
#include "utils/utm_converter.hpp"

#ifndef UTM_H
    extern int  LatLonToUTMXY(double lat, double lon, int zone, double& x, double& y);
    extern void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double& lat, double& lon);
#endif

int testLocRawGPS2UTM(const dg::LatLon& x, const dg::Point2& sol, bool verbose = true)
{
    dg::Point2 y;
    int zone = LatLonToUTMXY(x.lat, x.lon, -1, y.x, y.y);
    if (verbose) std::cout << "[VERBOSE] LatLon = ("  << x.lat << ", " << x.lon << ") --> UTM = " << y << ", Zone = " << zone << std::endl;
    if (sol.x >= 0 || sol.y >= 0)
    {
        VVS_CHECK_RANGE(y.x, sol.x, 0.1);
        VVS_CHECK_RANGE(y.y, sol.y, 0.1);
    }
    return 0;
}

int testLocRawUTM2GPS(const dg::Point2& x, int zone, bool is_south, const dg::LatLon& sol, bool verbose = true)
{
    dg::LatLon y;
    UTMXYToLatLon(x.x, x.y, zone, is_south, y.lat, y.lon);
    y.lat *= 180 / CV_PI; // [rad] to [deg]
    y.lon *= 180 / CV_PI; // [rad] to [deg]
    if (verbose) std::cout << "[VERBOSE] UTM = " << x << ", Zone = " << zone << " --> LatLon = (" << y.lat << ", " << y.lon << ")" << std::endl;
    if (sol.lat >= 0 || sol.lon >= 0)
    {
        VVS_CHECK_RANGE(y.lat, sol.lat, 0.1);
        VVS_CHECK_RANGE(y.lon, sol.lon, 0.1);
    }
    return 0;
}

int testLocUTMConverter()
{
    dg::UTMConverter converter;

    // Check reference
    dg::LatLon refer_ll = dg::LatLon(37, 127);
    dg::Point2UTM refer_utm = converter.cvtLatLon2UTM(refer_ll);

    VVS_CHECK_TRUE(converter.setReference(refer_ll));
    {
        dg::Point2UTM refer = converter.getReference();
        VVS_CHECK_EQUL(refer_utm.is_south, refer.is_south);
        VVS_CHECK_EQUL(refer_utm.zone, refer.zone);
        VVS_CHECK_NEAR(refer_utm.x, refer.x);
        VVS_CHECK_NEAR(refer_utm.y, refer.y);
    }

    VVS_CHECK_TRUE(converter.setReference(refer_utm));
    {
        dg::Point2UTM refer = converter.getReference();
        VVS_CHECK_EQUL(refer_utm.is_south, refer.is_south);
        VVS_CHECK_EQUL(refer_utm.zone, refer.zone);
        VVS_CHECK_NEAR(refer_utm.x, refer.x);
        VVS_CHECK_NEAR(refer_utm.y, refer.y);
    }

    // Check several points
    dg::Point2 origin = converter.toMetric(refer_ll);
    VVS_CHECK_NEAR(origin.x, 0);
    VVS_CHECK_NEAR(origin.y, 0);

    dg::Point2 p = converter.toMetric(dg::LatLon(38, 128));
    VVS_CHECK_TRUE(p.x > 0);
    VVS_CHECK_TRUE(p.y > 0);

    return 0;
}

#endif // End of '__TEST_LOCALIZER_GPS2UTM__'
