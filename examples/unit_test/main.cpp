#include "test_core_graph.hpp"
#include "test_core_type.hpp"
#include "test_localizer_road.hpp"
#include "test_localizer_simple.hpp"
#include "test_localizer_example.hpp"
#include "test_localizer_gps2utm.hpp"

int main()
{
    // Please change 'NUN' to 'RUN' if you want run a test

    // Test 'core' module
    // 1. Test 'dg::DirectedGraph'
    VVS_NUN_TEST(testDirectedGraphPtr());
    VVS_NUN_TEST(testDirectedGraphItr());

    // 2. Test basic data structures
    VVS_NUN_TEST(testCoreLatLon());
    VVS_NUN_TEST(testCorePolar2());
    VVS_NUN_TEST(testCorePoint2ID());

    // 3. Test 'dg::Map' and its related
    VVS_NUN_TEST(testCoreNodeInfo());
    VVS_NUN_TEST(testCoreEdgeInfo());
    VVS_NUN_TEST(testCoreMap());


    // Test 'localizer' module
    // 1. Test simple map, painter, and localizer
    VVS_NUN_TEST(testLocSimpleRoadMap());
    VVS_NUN_TEST(testLocSimpleRoadPainter());
    VVS_NUN_TEST(testLocSimpleLocalizer());
    VVS_NUN_TEST(testLocMap2SimpleRoadMap());

    // 2. Test GPS and UTM conversion
    VVS_NUN_TEST(testLocRawGPS2UTM(dg::LatLon(38, 128), dg::Point2(412201.58, 4206286.76))); // Zone: 52S
    VVS_NUN_TEST(testLocRawGPS2UTM(dg::LatLon(37, 127), dg::Point2(322037.81, 4096742.06))); // Zone: 52S
    VVS_NUN_TEST(testLocRawUTM2GPS(dg::Point2(412201.58, 4206286.76), 52, false, dg::LatLon(38, 128)));
    VVS_NUN_TEST(testLocRawUTM2GPS(dg::Point2(322037.81, 4096742.06), 52, false, dg::LatLon(37, 127)));
    VVS_NUN_TEST(testLocRawUTM2GPS(dg::Point2(0, 0), 52, false, dg::LatLon(-1, -1))); // Print the origin of the Zone 52
    VVS_NUN_TEST(testLocUTMConverter());

    return 0;
}