#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "lib/graph.h"

using ::testing::ElementsAre;

TEST(TestSearch, SimpleGraph) {
    Graph g;
    g.AddVertex(-1, 0);
    g.AddVertex(0, 1);
    g.AddVertex(1, 1);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);

    for (int u = 0; u < 2; ++u) {
        for (int v = 0; v < 2; ++v) {
            if (u != v) {
                EXPECT_THAT(g.Search(u, v), ElementsAre(u, v));
            } else {
                EXPECT_THAT(g.Search(u, v), ElementsAre(u));
            }
        }
    }
}

TEST(TestSearch, MediumGraph) {
    Graph g;
    g.AddVertex(-1, 0);
    g.AddVertex(0, 1);
    g.AddVertex(1, 1);
    g.AddVertex(2, 0);
    g.AddVertex(1, -1);
    g.AddVertex(0, -1);
    g.AddVertex(3, 1);
    g.AddVertex(2, 1);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    g.AddEdge(0, 3);
    g.AddEdge(0, 5);
    g.AddEdge(4, 3);
    g.AddEdge(4, 5);
    g.AddEdge(3, 6);
    g.AddEdge(6, 7);

    EXPECT_THAT(g.Search(0, 2), ElementsAre(0, 2));
    EXPECT_THAT(g.Search(0, 3), ElementsAre(0, 3));
    EXPECT_THAT(g.Search(1, 3), ElementsAre(1, 0, 3));
    EXPECT_THAT(g.Search(2, 3), ElementsAre(2, 0, 3));
    EXPECT_THAT(g.Search(0, 4), ElementsAre(0, 5, 4));
    EXPECT_THAT(g.Search(1, 4), ElementsAre(1, 0, 5, 4));
    EXPECT_THAT(g.Search(2, 4), ElementsAre(2, 0, 5, 4));
    EXPECT_THAT(g.Search(1, 5), ElementsAre(1, 0, 5));
    EXPECT_THAT(g.Search(2, 5), ElementsAre(2, 0, 5));
    EXPECT_THAT(g.Search(3, 5), ElementsAre(3, 4, 5));
    EXPECT_THAT(g.Search(0, 6), ElementsAre(0, 3, 6));
    EXPECT_THAT(g.Search(1, 6), ElementsAre(1, 0, 3, 6));
    EXPECT_THAT(g.Search(2, 6), ElementsAre(2, 0, 3, 6));
    EXPECT_THAT(g.Search(4, 6), ElementsAre(4, 3, 6));
    EXPECT_THAT(g.Search(5, 6), ElementsAre(5, 4, 3, 6));
    EXPECT_THAT(g.Search(0, 7), ElementsAre(0, 3, 6, 7));
    EXPECT_THAT(g.Search(1, 7), ElementsAre(1, 0, 3, 6, 7));
    EXPECT_THAT(g.Search(2, 7), ElementsAre(2, 0, 3, 6, 7));
    EXPECT_THAT(g.Search(3, 7), ElementsAre(3, 6, 7));
    EXPECT_THAT(g.Search(4, 7), ElementsAre(4, 3, 6, 7));
    EXPECT_THAT(g.Search(5, 7), ElementsAre(5, 4, 3, 6, 7));
}