#include <gtest/gtest.h>
#include <unordered_map>

#include "../include/graph types/edge_list.hpp"
#include "../include/graph types/adjacency_list.hpp"
#include "../include/graph types/adjacency_matrix.hpp"
#include "../include/first searches/breadth_first_search.hpp"
#include "../include/first searches/depth_first_search.hpp"
#include "../include/connectivity/bridges_and_points_of_articulation.hpp"

int a = 1, b = 2, c = 3, d = 4, e = 5, f = 6, g = 7, h = 8;

template <typename Graph>
class BFSPush : public BFSVisitor<Graph> {
 public:
  BFSPush(int* res) : res_(res) {}

  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
    const Graph& graph) { *res_ += *vertex; }

 private:
  int* res_;
};

template <typename Graph>
class DFSPush : public DFSVisitor<Graph> {
 public:
  DFSPush(int* res) : res_(res) {}

  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
    const Graph& graph) { *res_ += *vertex; }

 private:
  int* res_;
};

//------------BFS tests-------------------

TEST(BFSTests, EdgeListTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  BreadthFirstSearch(graph, &a, BFSPush<EdgeList<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

TEST(BFSTests, AdjacencyListTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  BreadthFirstSearch(graph, &a, BFSPush<AdjacencyList<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

TEST(BFSTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false> graph(8);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  BreadthFirstSearch(graph, &a, BFSPush<AdjacencyMatrix<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

//------------DFS tests-------------------

TEST(DFSTests, EdgeListTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  DepthFirstSearch(graph, &a, DFSPush<EdgeList<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

TEST(DFSTests, AdjacencyListTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  DepthFirstSearch(graph, &a, DFSPush<AdjacencyList<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

TEST(DFSTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false> graph(8);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&b, &e);
  graph.add_edge(&b, &f);
  graph.add_edge(&c, &g);
  graph.add_edge(&d, &h);

  std::unordered_map<int*, Colour> colours;
  int res = 0;
  DepthFirstSearch(graph, &a, DFSPush<AdjacencyMatrix<int, false>>(&res), colours);
  ASSERT_EQ(res, 36);
}

//---------Bridges tests----------------

TEST(BridgesTests, EdgeListTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindBridges(graph);
  std::pair test = {&a, &d};
  std::pair test_rev = {&d, &a};
  for(auto bridge : res) {
    ASSERT_TRUE(bridge == test || bridge == test_rev);
  }
}

TEST(BridgesTests, AdjacencyListTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindBridges(graph);
  std::pair test = {&a, &d};
  std::pair test_rev = {&d, &a};
  for(auto bridge : res) {
    ASSERT_TRUE(bridge == test || bridge == test_rev);
  }
}

TEST(BridgesTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false> graph(8);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindBridges(graph);
  std::pair test = {&a, &d};
  std::pair test_rev = {&d, &a};
  for(auto bridge : res) {
    ASSERT_TRUE(bridge == test || bridge == test_rev);
  }
}

//--PointsOfArticulation tests---------------

TEST(PointsOfArticulationTests, EdgeListTest1) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindPointsOfArticulation(graph);
  std::unordered_set<int*> test = {&a, &d};
  for(auto point : res) {
    ASSERT_TRUE(point == &a || point == &d);
  }
}

TEST(PointsOfArticulationTests, AdjacencyListTest1) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindPointsOfArticulation(graph);
  std::unordered_set<int*> test = {&a, &d};
  for(auto point : res) {
    ASSERT_TRUE(point == &a || point == &d);
  }
}

TEST(PointsOfArticulationTests, AdjacencyMatrixTest1) {
  AdjacencyMatrix<int, false> graph(8);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&b, &c);
  graph.add_edge(&a, &c);
  graph.add_edge(&a, &d);
  graph.add_edge(&d, &e);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);

  auto res = FindPointsOfArticulation(graph);
  std::unordered_set<int*> test = {&a, &d};
  for(auto point : res) {
    ASSERT_TRUE(point == &a || point == &d);
  }
}

TEST(PointsOfArticulationTests, EdgeListTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&c, &b);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);
  graph.add_edge(&e, &d);

  auto res = FindPointsOfArticulation(graph);
  for (auto point : res)
    std::cout << *point << '\n';
  ASSERT_TRUE(res.empty());
}

TEST(PointsOfArticulationTests, AdjacencyListTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&c, &b);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);
  graph.add_edge(&e, &d);

  auto res = FindPointsOfArticulation(graph);
  ASSERT_TRUE(res.empty());
}

TEST(PointsOfArticulationTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false> graph(8);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_edge(&a, &b);
  graph.add_edge(&a, &c);
  graph.add_edge(&c, &b);
  graph.add_edge(&d, &f);
  graph.add_edge(&f, &e);
  graph.add_edge(&e, &d);

  auto res = FindPointsOfArticulation(graph);
  ASSERT_TRUE(res.empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}