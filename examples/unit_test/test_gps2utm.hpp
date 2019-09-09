#ifndef __TEST_GPS_2_UTM__
#define __TEST_GPS_2_UTM__

#include "data_structure.hpp"

int testLatLon()
{
    dg::LatLon a, b(1, 2);
    VVS_CHECK_TRUE(a.lat == 0);
    VVS_CHECK_TRUE(a.lon == 0);
    VVS_CHECK_TRUE(a.x == 0);
    VVS_CHECK_TRUE(a.y == 0);
    VVS_CHECK_TRUE(b.lat == 1);
    VVS_CHECK_TRUE(b.lon == 2);
    VVS_CHECK_TRUE(b.x == 1);
    VVS_CHECK_TRUE(b.y == 2);

    a.lat = 1;
    a.lon = 2;
    VVS_CHECK_TRUE(a.lat == 1);
    VVS_CHECK_TRUE(a.lon == 2);
    VVS_CHECK_TRUE(a.x == 1);
    VVS_CHECK_TRUE(a.y == 2);
    VVS_CHECK_TRUE(a == b);

    b = dg::Point2(3, 4);
    VVS_CHECK_TRUE(b.lat == 3);
    VVS_CHECK_TRUE(b.lon == 4);
    VVS_CHECK_TRUE(b.x == 3);
    VVS_CHECK_TRUE(b.y == 4);
    VVS_CHECK_TRUE(a != b);

    a = 2 * b - b;
    VVS_CHECK_TRUE(a.lat == 3);
    VVS_CHECK_TRUE(a.lon == 4);
    VVS_CHECK_TRUE(a.x == 3);
    VVS_CHECK_TRUE(a.y == 4);
    VVS_CHECK_TRUE(a == b);

    return 0;
}

#endif // End of '__TEST_GPS_2_UTM__'
