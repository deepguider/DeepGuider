#include "test_directed_graph.hpp"
#include "test_simple_road.hpp"

int main()
{
    VVS_NUN_TEST(testDirectedGraph());
    VVS_NUN_TEST(testPoint2ID());
    VVS_NUN_TEST(testSimpleRoadMap());
    VVS_RUN_TEST(testSimpleRoadPainter(true));

    return 0;
}