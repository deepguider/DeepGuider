#include "test_core_type.hpp"
#include "test_localizer_graph.hpp"
#include "test_localizer_gps2utm.hpp"
#include "test_localizer_road.hpp"
#include "test_localizer_simple.hpp"
#include "test_localizer_ekf.hpp"
#include "test_localizer_etri.hpp"
#include "run_localizer.hpp"

int runUnitTest()
{
    // Test 'core' module
    // 1. Test basic data structures
    VVS_RUN_TEST(testCoreLatLon());
    VVS_RUN_TEST(testCorePolar2());
    VVS_RUN_TEST(testCorePoint2ID());

    // 2. Test 'dg::Map' and its related
    VVS_RUN_TEST(testCoreNode());
    VVS_RUN_TEST(testCoreEdge());
    VVS_RUN_TEST(testCoreMap());
    VVS_RUN_TEST(testCorePath());


    // Test 'localizer' module
    // 1. Test GPS and UTM conversion
    VVS_RUN_TEST(testLocRawGPS2UTM(dg::LatLon(38, 128), dg::Point2(412201.58, 4206286.76))); // Zone: 52S
    VVS_RUN_TEST(testLocRawGPS2UTM(dg::LatLon(37, 127), dg::Point2(322037.81, 4096742.06))); // Zone: 52S
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(412201.58, 4206286.76), 52, false, dg::LatLon(38, 128)));
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(322037.81, 4096742.06), 52, false, dg::LatLon(37, 127)));
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(0, 0), 52, false, dg::LatLon(-1, -1))); // Print the origin of the Zone 52
    VVS_RUN_TEST(testLocUTMConverter());

    // 2. Test 'dg::DirectedGraph'
    VVS_RUN_TEST(testDirectedGraphPtr());
    VVS_RUN_TEST(testDirectedGraphItr());

    // 3. Test 'dg::RoadMap' and 'dg::GraphPainter'
    VVS_RUN_TEST(testLocRoadMap());
    VVS_RUN_TEST(testLocRoadPainter());

    // 4. Test localizers
    VVS_RUN_TEST(testLocBaseDist2());
    VVS_RUN_TEST(testLocSimple());

    VVS_RUN_TEST(testLocEKFGPS());
    VVS_RUN_TEST(testLocEKFGyroGPS());
    VVS_RUN_TEST(testLocEKFLocClue());

    VVS_RUN_TEST(testLocETRIMap2RoadMap());
    VVS_RUN_TEST(testLocETRISyntheticMap());
    VVS_RUN_TEST(testLocETRIRealMap());
    VVS_RUN_TEST(testLocCOEXRealMap());

    return 0;
}

int main()
{
    //return runUnitTest();
    return runLocalizer();
}
