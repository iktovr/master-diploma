#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "lib/graph.h"

using ::testing::UnorderedElementsAre;

using namespace lib;

// ---------------------------------------------------------------------------
// Graph construction
// ---------------------------------------------------------------------------

TEST(GraphConstruction, AddVerticesAndEdges) {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(3.0, 4.0, Graph::Vertex::delivery);
    g.AddVertex(6.0, 0.0);
    g.AddEdge(0, 1);
    g.AddEdge(1, 2, /*narrow=*/true);

    ASSERT_EQ(g.vertices.size(), 3u);
    EXPECT_EQ(g.vertices[0].id, 0);
    EXPECT_EQ(g.vertices[1].id, 1);
    EXPECT_EQ(g.vertices[2].id, 2);
    EXPECT_EQ(g.vertices[0].type, Graph::Vertex::base);
    EXPECT_EQ(g.vertices[1].type, Graph::Vertex::delivery);
    EXPECT_EQ(g.vertices[2].type, Graph::Vertex::none);

    // Edge 0-1: wide, length == 5
    ASSERT_TRUE(g.edges[0].count(1));
    EXPECT_FALSE(g.edges[0].at(1).narrow);
    EXPECT_DOUBLE_EQ(g.edges[0].at(1).length, 5.0);
    EXPECT_DOUBLE_EQ(g.edges[1].at(0).length, 5.0);

    // Edge 1-2: narrow, symmetric
    ASSERT_TRUE(g.edges[1].count(2));
    EXPECT_TRUE(g.edges[1].at(2).narrow);
    EXPECT_TRUE(g.edges[2].at(1).narrow);
}

// ---------------------------------------------------------------------------
// Graph::Distance
// ---------------------------------------------------------------------------

TEST(GraphDistance, KnownDistances) {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(3.0, 4.0);

    EXPECT_DOUBLE_EQ(g.Distance(0, 1), 5.0);
    EXPECT_DOUBLE_EQ(g.Distance(1, 0), 5.0);
    EXPECT_DOUBLE_EQ(g.Distance(0, 0), 0.0);
}

// ---------------------------------------------------------------------------
// Graph::GetVertices
// ---------------------------------------------------------------------------

TEST(GraphGetVertices, FilterByType) {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 0.0);
    g.AddVertex(2.0, 0.0, Graph::Vertex::delivery);
    g.AddVertex(3.0, 0.0, Graph::Vertex::base);

    EXPECT_THAT(g.GetVertices(),                        UnorderedElementsAre(0, 1, 2, 3));
    EXPECT_THAT(g.GetVertices(Graph::Vertex::base),     UnorderedElementsAre(0, 3));
    EXPECT_THAT(g.GetVertices(Graph::Vertex::delivery), UnorderedElementsAre(2));
    EXPECT_THAT(g.GetVertices(Graph::Vertex::none),     UnorderedElementsAre(1));
}

// ---------------------------------------------------------------------------
// Graph::Width / Height / Centroid
// ---------------------------------------------------------------------------

TEST(GraphGeometry, WidthHeightCentroid) {
    Graph g;
    g.AddVertex(-2.0, -1.0);
    g.AddVertex(3.0, 4.0);

    // Width  = |-2| + |3| = 5
    // Height = |-1| + |4| = 5
    // Centroid = ((-2+3)/2, (-1+4)/2) = (0.5, 1.5)
    EXPECT_DOUBLE_EQ(g.Width(), 5.0);
    EXPECT_DOUBLE_EQ(g.Height(), 5.0);
    EXPECT_DOUBLE_EQ(g.Centroid().x(), 0.5);
    EXPECT_DOUBLE_EQ(g.Centroid().y(), 1.5);
}

// ---------------------------------------------------------------------------
// Graph::LoadFrom*
// ---------------------------------------------------------------------------

TEST(GraphInput, LoadFromFile) {
    Graph g = Graph::LoadFromFile("tests/data/graph.txt");

    ASSERT_EQ(g.vertices.size(), 3u);
    EXPECT_EQ(g.vertices[0].type, Graph::Vertex::base);
    EXPECT_EQ(g.vertices[1].type, Graph::Vertex::none);
    EXPECT_EQ(g.vertices[2].type, Graph::Vertex::delivery);

    // Edge 0-1: narrow
    ASSERT_TRUE(g.edges[0].contains(1));
    EXPECT_TRUE(g.edges[0].at(1).narrow);
    ASSERT_TRUE(g.edges[1].contains(0));
    EXPECT_TRUE(g.edges[1].at(0).narrow);

    // Edge 0-2: not narrow
    ASSERT_TRUE(g.edges[0].contains(2));
    EXPECT_FALSE(g.edges[0].at(2).narrow);
    ASSERT_TRUE(g.edges[2].contains(0));
    EXPECT_FALSE(g.edges[2].at(0).narrow);

    // Edge 1-2: not narrow
    ASSERT_TRUE(g.edges[1].contains(2));
    EXPECT_FALSE(g.edges[1].at(2).narrow);
    ASSERT_TRUE(g.edges[2].contains(1));
    EXPECT_FALSE(g.edges[2].at(1).narrow);
}

TEST(GraphInput, LoadFromGeoJsonFile) {
    Graph g = Graph::LoadFromGeoJsonFile("tests/data/graph.geojson");

    ASSERT_EQ(g.vertices.size(), 3u);
    EXPECT_EQ(g.GetVertices(Graph::Vertex::base).size(), 1u);
    EXPECT_EQ(g.GetVertices(Graph::Vertex::none).size(), 1u);
    EXPECT_EQ(g.GetVertices(Graph::Vertex::delivery).size(), 1u);

    // Find vertex ids by type
    int base_id = g.GetVertices(Graph::Vertex::base)[0];
    int none_id = g.GetVertices(Graph::Vertex::none)[0];
    int deliv_id = g.GetVertices(Graph::Vertex::delivery)[0];

    // Edge base-none
    ASSERT_TRUE(g.edges[base_id].contains(none_id));
    EXPECT_FALSE(g.edges[base_id].at(none_id).narrow);
    ASSERT_TRUE(g.edges[none_id].contains(base_id));
    EXPECT_FALSE(g.edges[none_id].at(base_id).narrow);

    // Edge base-delivery
    ASSERT_TRUE(g.edges[base_id].contains(deliv_id));
    EXPECT_FALSE(g.edges[base_id].at(deliv_id).narrow);
    ASSERT_TRUE(g.edges[deliv_id].contains(base_id));
    EXPECT_FALSE(g.edges[deliv_id].at(base_id).narrow);

    // Edge none-delivery: narrow
    ASSERT_TRUE(g.edges[none_id].contains(deliv_id));
    EXPECT_TRUE(g.edges[none_id].at(deliv_id).narrow);
    ASSERT_TRUE(g.edges[deliv_id].contains(none_id));
    EXPECT_TRUE(g.edges[deliv_id].at(none_id).narrow);
}
