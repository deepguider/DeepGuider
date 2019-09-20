#ifndef __TEST_DIRECTED_GRAPH__
#define __TEST_DIRECTED_GRAPH__

#include "vvs.h"
#include "dg_core.hpp"
#include <string>

typedef dg::DirectedGraph<std::string, int> FamilyTree;

int testDirectedGraphPtr(const char* file = "test_directed_graph.txt")
{
    // Build a family tree
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

    // Check connectivities
    VVS_CHECK_EQUL(family.getEdgeCost(person[3], person[8]), 1);
    VVS_CHECK_EQUL(family.getEdgeCost(person[6], person[8]), -1);
    VVS_CHECK_TRUE(family.isConnected(person[0], person[2]));
    VVS_CHECK_TRUE(family.isConnected(person[2], person[5]) == false);

    // Add and remove edges and nodes
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

    // Write the family to the file
    FILE* fid = fopen(file, "wt");
    VVS_CHECK_TRUE(fid != NULL);
    for (FamilyTree::NodeItrConst node = family.getHeadNodeConst(); node != family.getTailNodeConst(); node++)
    {
        fprintf(fid, "* Node: %s\n", node->data.c_str());
        for (FamilyTree::EdgeItrConst edge = family.getHeadEdgeConst(node); edge != family.getTailEdgeConst(node); edge++)
            fprintf(fid, "  Is connected to %s (%s)\n", edge->to->data.c_str(), (edge->cost > 0) ? "Child" : "Marriage");
    }
    fclose(fid);

    // Remove all
    VVS_CHECK_EQUL(family.countNodes(), 10);
    VVS_CHECK_TRUE(family.removeAll());
    VVS_CHECK_EQUL(family.countNodes(), 0);

    return 0;
}

int testDirectedGraphItr()
{
    // Build a family tree
    FamilyTree family;
    VVS_CHECK_TRUE(family.addNode("CS") != NULL);
    VVS_CHECK_TRUE(family.addNode("LK") != NULL);
    VVS_CHECK_TRUE(family.addNode("CJ") != NULL);
    VVS_CHECK_TRUE(family.addNode("CW") != NULL);
    VVS_CHECK_TRUE(family.addEdge("CS", "LK", 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge("LK", "CS", 0) != NULL);
    VVS_CHECK_TRUE(family.addEdge("CS", "CJ", 1) != NULL);
    VVS_CHECK_TRUE(family.addEdge("CS", "CW", 1) != NULL);

    // Test const_iterators
    FamilyTree::NodeItrConst const_cs = family.getNodeConst("CS");
    FamilyTree::NodeItrConst const_lk = family.getNodeConst("LK");
    FamilyTree::NodeItrConst const_cj = family.getNodeConst("CJ");
    FamilyTree::NodeItrConst const_cw = family.getNodeConst("CW");

    VVS_CHECK_TRUE(const_cs != family.getTailNodeConst() && const_cs->data == "CS");
    VVS_CHECK_TRUE(const_lk != family.getTailNodeConst() && const_lk->data == "LK");
    VVS_CHECK_TRUE(const_cj != family.getTailNodeConst() && const_cj->data == "CJ");
    VVS_CHECK_TRUE(const_cw != family.getTailNodeConst() && const_cw->data == "CW");

    FamilyTree::EdgeItrConst const_cs2lk = family.getEdgeConst(const_cs, const_lk);
    VVS_CHECK_TRUE(const_cs2lk != family.getTailEdgeConst(const_cs));
    VVS_CHECK_EQUL(family.getEdgeCost(const_cs, const_lk), 0);
    VVS_CHECK_TRUE(family.isConnected(const_cs, const_lk) == true);

    FamilyTree::EdgeItrConst const_lk2cj = family.getEdgeConst(const_lk, const_cj);
    VVS_CHECK_TRUE(const_lk2cj == family.getTailEdgeConst(const_lk));
    VVS_CHECK_EQUL(family.getEdgeCost(const_lk, const_cj), -1);
    VVS_CHECK_TRUE(family.isConnected(const_lk, const_cj) == false);

    // Test (normal) iterators
    FamilyTree::NodeItr cs = family.getHeadNode();
    FamilyTree::NodeItr lk = family.getHeadNode(); lk++;
    FamilyTree::NodeItr cj = family.getHeadNode(); cj++; cj++;
    FamilyTree::NodeItr cw = family.getHeadNode(); cw++; cw++; cw++;

    VVS_CHECK_TRUE(cs != family.getTailNode() && cs->data == "CS");
    VVS_CHECK_TRUE(lk != family.getTailNode() && lk->data == "LK");
    VVS_CHECK_TRUE(cj != family.getTailNode() && cj->data == "CJ");
    VVS_CHECK_TRUE(cw != family.getTailNode() && cw->data == "CW");

    FamilyTree::Edge* cs2lk = family.getEdge(cs, lk);
    VVS_CHECK_TRUE(cs2lk != NULL);
    VVS_CHECK_EQUL(family.getEdgeCost(cs, lk), 0);
    VVS_CHECK_TRUE(family.isConnected(cs, lk) == true);

    FamilyTree::Edge* lk2cj = family.getEdge(lk, cj);
    VVS_CHECK_TRUE(lk2cj == NULL);
    VVS_CHECK_EQUL(family.getEdgeCost(lk, cj), -1);
    VVS_CHECK_TRUE(family.isConnected(lk, cj) == false);

    // Test 'copyTo'
    FamilyTree copy;
    VVS_CHECK_TRUE(family.copyTo(&copy));
    VVS_CHECK_EQUL(copy.countNodes(), 4);
    VVS_CHECK_EQUL(copy.countEdges("CS"), 3);
    VVS_CHECK_EQUL(copy.countEdges("LK"), 1);
    VVS_CHECK_EQUL(copy.countEdges("CJ"), 0);
    VVS_CHECK_EQUL(copy.countEdges("CW"), 0);
    VVS_CHECK_EQUL(copy.getEdgeCost("CS", "LK"), 0);
    VVS_CHECK_EQUL(copy.getEdgeCost("LK", "CS"), 0);
    VVS_CHECK_EQUL(copy.getEdgeCost("CS", "CJ"), 1);
    VVS_CHECK_EQUL(copy.getEdgeCost("CS", "CW"), 1);

    return 0;
}

#endif // End of '__TEST_DIRECTED_GRAPH__'