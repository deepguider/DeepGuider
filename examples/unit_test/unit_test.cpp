#include "test_directed_graph.hpp"
#include "test_simple_road_map.hpp"

int main()
{
    VVS_NUN_TEST(testDirectedGraph());
    VVS_RUN_TEST(testPointID());

    return 0;
}