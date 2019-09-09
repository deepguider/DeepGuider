#include "test_directed_graph.hpp"
#include "test_simple_road.hpp"
#include "test_simple_localizer.hpp"
#include "test_gps2utm.hpp"

int main()
{
    // Please change 'NUN' to 'RUN' if you want run a test

    // Test basic definitions
    VVS_NUN_TEST(testDirectedGraphPtr());
    VVS_NUN_TEST(testDirectedGraphItr());

    // Test simple cases
    VVS_NUN_TEST(testPoint2ID());
    VVS_NUN_TEST(testSimpleRoadMap());
    VVS_NUN_TEST(testSimpleRoadPainter(true));
    VVS_NUN_TEST(testSimpleMetricLocalizer(true));

    // Test GPS and UTM conversion
    VVS_RUN_TEST(testLatLon());

    return 0;
}