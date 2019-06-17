#include "test_directed_graph.hpp"
#include "test_simple_road_map.hpp"

int main()
{
    VVS_NUN_TEST(testDirectedGraph());
    VVS_NUN_TEST(testPoint2ID());
    VVS_RUN_TEST(testSimpleRoadMap());

    return 0;
}