#ifndef __TEST_GPS_2_UTM__
#define __TEST_GPS_2_UTM__

#include "dg_core.hpp"

int testLocLatLon()
{
    dg::LonLat a, b(1, 2);
    VVS_CHECK_TRUE(a.lon == 0);
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.x == 0);
    VVS_CHECK_TRUE(a.y == 0);
    VVS_CHECK_TRUE(b.lon == 1);
    VVS_CHECK_TRUE(b.lat == 2);
    VVS_CHECK_TRUE(b.x == 1);
    VVS_CHECK_TRUE(b.y == 2);

    a.lon = 1;
    a.lat = 2;
    VVS_CHECK_TRUE(a.lon == 1);
    VVS_CHECK_TRUE(a.lat == 2);
    VVS_CHECK_TRUE(a.x == 1);
    VVS_CHECK_TRUE(a.y == 2);
    VVS_CHECK_TRUE(a == b);

    b = dg::Point2(3, 4);
    VVS_CHECK_TRUE(b.lon == 3);
    VVS_CHECK_TRUE(b.lat == 4);
    VVS_CHECK_TRUE(b.x == 3);
    VVS_CHECK_TRUE(b.y == 4);
    VVS_CHECK_TRUE(a != b);

    a = 2 * b - b;
    VVS_CHECK_TRUE(a.lon == 3);
    VVS_CHECK_TRUE(a.lat == 4);
    VVS_CHECK_TRUE(a.x == 3);
    VVS_CHECK_TRUE(a.y == 4);
    VVS_CHECK_TRUE(a == b);

    return 0;
}

#ifndef UTM_H
    extern int  LatLonToUTMXY(double lat, double lon, int zone, double& x, double& y);
    extern void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double& lat, double& lon);
#endif

int testLocRawGPS2UTM(const dg::LonLat& x, const dg::Point2& sol, bool verbose = true)
{
    dg::Point2 y;
    int zone = LatLonToUTMXY(x.lat, x.lon, -1, y.x, y.y);
    if (verbose) std::cout << "[VERBOSE] LonLat = " << x << " --> UTM = " << y << ", Zone = " << zone << std::endl;
    if (sol.x >= 0 || sol.y >= 0)
    {
        VVS_CHECK_RANGE(y.x, sol.x, 0.1);
        VVS_CHECK_RANGE(y.y, sol.y, 0.1);
    }
    return 0;
}

int testLocRawUTM2GPS(const dg::Point2& x, int zone, bool is_south, const dg::LonLat& sol, bool verbose = true)
{
    dg::LonLat y;
    UTMXYToLatLon(x.x, x.y, zone, is_south, y.lat, y.lon);
    y *= 180 / CV_PI; // [rad] to [deg]
    if (verbose) std::cout << "[VERBOSE] UTM = " << x << ", Zone = " << zone << " --> LonLat = " << y << std::endl;
    if (sol.x >= 0 || sol.y >= 0)
    {
        VVS_CHECK_RANGE(y.lon, sol.lon, 0.1);
        VVS_CHECK_RANGE(y.lat, sol.lat, 0.1);
    }
    return 0;
}

#endif // End of '__TEST_GPS_2_UTM__'
