#ifndef __TEST_DIRECTED_GRAPH__
#define __TEST_DIRECTED_GRAPH__

#include "vvs.h"
#include "directed_graph.hpp"
#include <string>

int testDirectedGraph(const char* filename = "test_directed_graph.txt")
{
    typedef dg::DirectedGraph<std::string, int> FamilyTree;

    FamilyTree family;
    VVS_CHECK_TRUE(family.addNode("Dangeun") != NULL);
    FamilyTree::Node* person[] =
    {
        /* 0 */ family.addNode("CY"),
        /* 1 */ family.addNode("GY"),
        /* 2 */ family.addNode("CH"),
        /* 3 */ family.addNode("CS"),
        /* 4 */ family.addNode("PS"),
        /* 5 */ family.addNode("PSY"),
        /* 6 */ family.addNode("LK"),
        /* 7 */ family.addNode("PSE"),
        /* 8 */ family.addNode("CJ"),
        /* 9 */ family.addNode("CW"),
    };
    VVS_CHECK_TRUE(family.getNode("CY") == person[0]);
    VVS_CHECK_TRUE(family.getNode("Guest") == NULL);

    VVS_CHECK_TRUE(family.addEdge(person[0], person[1], 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(person[1], person[0], 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(person[0], person[2], 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge(person[0], person[3], 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("CH"), family.getNode("PS"), 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("PS"), family.getNode("CH"), 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("PS"), family.getNode("PSY"), 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("CS"), family.getNode("LK"), 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("LK"), family.getNode("CS"), 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("PS"), family.getNode("PSE"), 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("CS"), family.getNode("CJ"), 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge(family.getNode("CS"), family.getNode("CW"), 1) != NULL);
    VVS_CHECK_TRUE(family.getEdge("CY", "CS") != NULL);
    VVS_CHECK_TRUE(family.getEdge("CS", "CY") == NULL);
    VVS_CHECK_TRUE(family.getEdge(person[0], person[1]) != NULL);
    VVS_CHECK_TRUE(family.getEdge(person[1], person[0]) != NULL);
    VVS_CHECK_EQUL(family.getEdgeCost(person[3], person[8]), 1);
    VVS_CHECK_EQUL(family.getEdgeCost(person[6], person[8]), -1);
    VVS_CHECK_TRUE(family.isConnected(person[0], person[2]));
    VVS_CHECK_TRUE(family.isConnected(person[2], person[5]) == false);

    FamilyTree::Node* gf = family.addNode("Grand Father");
    FamilyTree::Node* gm = family.addNode("Grand Mother");
    VVS_CHECK_TRUE(family.addEdge(gf, gm, 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(gm, gf, 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge(gf, person[0], 1) != NULL);
    VVS_CHECK_EQUL(family.countEdges(gf), 2);
    VVS_CHECK_EQUL(family.countEdges(gm), 1);
    VVS_CHECK_TRUE(family.removeNode(gf));
    VVS_CHECK_EQUL(family.countEdges(gm), 0);
    VVS_CHECK_TRUE(family.removeNode(gm));
    VVS_CHECK_TRUE(family.removeNode(family.getNode("Dangeun")));

    FILE* fid = fopen(filename, "wt");
    VVS_CHECK_TRUE(fid != NULL);
    for (FamilyTree::NodeItr node = family.getHeadNode(); node != family.getTailNode(); node++)
    {
        fprintf(fid, "* Node: %s\n", node->data.c_str());
        for (FamilyTree::EdgeItr edge = family.getHeadEdge(node); edge != family.getTailEdge(node); edge++)
            fprintf(fid, "  Is connected to %s (%s)\n", edge->to->data.c_str(), (edge->cost > 0) ? "Child" : "Marriage");
    }
    fclose(fid);

    VVS_CHECK_EQUL(family.countNodes(), 10);
    VVS_CHECK_TRUE(family.removeAll());
    VVS_CHECK_EQUL(family.countNodes(), 0);

    return 0;
}

#endif // End of '__TEST_DIRECTED_GRAPH__'