#include "test_directed_graph.hpp"
#include "test_simple_road.hpp"
#include "test_localizer.hpp"

int main()
{
    // Please change 'NUN' to 'RUN' if you want run a test

    // Test graph and my simple road map
    VVS_NUN_TEST(testDirectedGraph());
    VVS_NUN_TEST(testPoint2ID());
    VVS_NUN_TEST(testSimpleRoadMap());
    VVS_NUN_TEST(testSimpleRoadPainter(true));

    // Test metric and topometric localizers
    VVS_RUN_TEST(testLocalizerMetric());
    VVS_RUN_TEST(testLocalizerTopometric());

    return 0;
}