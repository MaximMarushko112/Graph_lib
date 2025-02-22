#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "graph types/adjacency_list.hpp"
#include "graph types/adjacency_matrix.hpp"
#include "graph types/edge_list.hpp"

int a = 1, b = 2, c = 3, d = 4, e = 5;
const int kInf = std::numeric_limits<int>::max();
std::vector<int> all_vertexes({1, 2, 3, 4});
std::vector<int> odd_vertexes({1, 3});

bool is_odd(int* n) { return *n % 2; }

template <typename Graph>
auto test_vertex_remove(Graph& graph,
                        typename Graph::vertex_descriptor&& vertex) {
  try {
    graph.remove_vertex(vertex);
  } catch (std::invalid_argument) {
    return false;
  }
  return true;
};

template <typename Graph>
auto test_add_wrong_weight_edge(Graph& graph,
                                typename Graph::vertex_descriptor&& start,
                                typename Graph::vertex_descriptor&& finish,
                                typename Graph::weight weight) {
  try {
    graph.add_edge(start, finish, weight);
  } catch (std::invalid_argument) {
    return false;
  }
  return true;
};

template <typename Graph>
auto test_add_wrong_unweight_edge(Graph& graph,
                                  typename Graph::vertex_descriptor&& start,
                                  typename Graph::vertex_descriptor&& finish) {
  try {
    graph.add_edge(start, finish);
  } catch (std::invalid_argument) {
    return false;
  }
  return true;
};

template <typename Graph>
auto test_remove_wrong_edge(Graph& graph,
                            typename Graph::vertex_descriptor&& start,
                            typename Graph::vertex_descriptor&& finish) {
  try {
    graph.remove_edge(start, finish);
  } catch (std::invalid_argument) {
    return false;
  }
  return true;
};

//--------EdgeList tests-----------------

TEST(EdgeListTests, VertexTest) {
  EdgeList<int, false> graph;

  ASSERT_TRUE(graph.add_vertex(&a));
  ASSERT_TRUE(graph.add_vertex(&b));
  ASSERT_TRUE(graph.add_vertex(&c));
  graph.add_edge(&a, &b);
  ASSERT_TRUE(test_vertex_remove(graph, &a));
  ASSERT_TRUE(test_vertex_remove(graph, &c));
  ASSERT_FALSE(test_vertex_remove(graph, &d));
}

TEST(EdgeListTests, DirectedNotWeightedEdgeTest) {
  EdgeList<int, true> directed_not_weighted;

  ASSERT_FALSE(directed_not_weighted.add_edge(&a, &b, 3));
  ASSERT_FALSE(test_add_wrong_unweight_edge(directed_not_weighted, &a, &b));
  directed_not_weighted.add_vertex(&a);
  directed_not_weighted.add_vertex(&b);
  directed_not_weighted.add_vertex(&c);
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &b));
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &c));
  ASSERT_EQ(directed_not_weighted.edge_weight(&a, &b), 1);
  ASSERT_EQ(directed_not_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(directed_not_weighted, &d, &b));
  ASSERT_TRUE(directed_not_weighted.remove_edge(&a, &b));
  ASSERT_FALSE(directed_not_weighted.remove_edge(&c, &a));
}

TEST(EdgeListTests, NotDirectedWeightedEdgeTest) {
  EdgeList<int, false, true, int> not_directed_weighted;

  ASSERT_FALSE(not_directed_weighted.add_edge(&a, &b));
  ASSERT_FALSE(test_add_wrong_weight_edge(not_directed_weighted, &a, &b, 2));
  not_directed_weighted.add_vertex(&a);
  not_directed_weighted.add_vertex(&b);
  not_directed_weighted.add_vertex(&c);
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &b, 3));
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &c, 2));
  ASSERT_EQ(not_directed_weighted.edge_weight(&a, &b), 3);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &a), 2);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(not_directed_weighted, &a, &d));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&a, &b));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&c, &a));
}

TEST(EdgeListTests, BasicIteratorsTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);

  std::vector<int> vertexes;
  for (auto v = graph.vertexes_begin(); v != graph.vertexes_end(); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.vertexes_begin(is_odd);
       !(v == graph.vertexes_end(is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

TEST(EdgeListTests, NeighboursTest) {
  EdgeList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &e);
  graph.add_edge(&b, &e);
  graph.add_edge(&c, &e);
  graph.add_edge(&d, &e);

  std::vector<int> vertexes;
  for (auto v = graph.neighbours_begin(&e); v != graph.neighbours_end(&e);
       ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.neighbours_begin(&e, is_odd);
       !(v == graph.neighbours_end(&e, is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

//--------AdjacencyList tests-----------------

TEST(AdjacencyListTests, VertexTest) {
  AdjacencyList<int, false> graph;

  ASSERT_TRUE(graph.add_vertex(&a));
  ASSERT_TRUE(graph.add_vertex(&b));
  ASSERT_TRUE(graph.add_vertex(&c));
  graph.add_edge(&a, &b);
  ASSERT_TRUE(test_vertex_remove(graph, &a));
  ASSERT_TRUE(test_vertex_remove(graph, &c));
  ASSERT_FALSE(test_vertex_remove(graph, &d));
}

TEST(AdjacencyListTests, DirectedNotWeightedEdgeTest) {
  AdjacencyList<int, true> directed_not_weighted;

  ASSERT_FALSE(directed_not_weighted.add_edge(&a, &b, 3));
  ASSERT_FALSE(test_add_wrong_unweight_edge(directed_not_weighted, &a, &b));
  directed_not_weighted.add_vertex(&a);
  directed_not_weighted.add_vertex(&b);
  directed_not_weighted.add_vertex(&c);
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &b));
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &c));
  ASSERT_EQ(directed_not_weighted.edge_weight(&a, &b), 1);
  ASSERT_EQ(directed_not_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(directed_not_weighted, &d, &b));
  ASSERT_TRUE(directed_not_weighted.remove_edge(&a, &b));
  ASSERT_FALSE(directed_not_weighted.remove_edge(&c, &a));
}

TEST(AdjacencyListTests, NotDirectedWeightedEdgeTest) {
  AdjacencyList<int, false, true, int> not_directed_weighted;

  ASSERT_FALSE(not_directed_weighted.add_edge(&a, &b));
  ASSERT_FALSE(test_add_wrong_weight_edge(not_directed_weighted, &a, &b, 2));
  not_directed_weighted.add_vertex(&a);
  not_directed_weighted.add_vertex(&b);
  not_directed_weighted.add_vertex(&c);
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &b, 3));
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &c, 2));
  ASSERT_EQ(not_directed_weighted.edge_weight(&a, &b), 3);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &a), 2);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(not_directed_weighted, &a, &d));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&a, &b));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&c, &a));
}

TEST(AdjacencyListTests, BasicIteratorsTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);

  std::vector<int> vertexes;
  for (auto v = graph.vertexes_begin(); v != graph.vertexes_end(); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.vertexes_begin(is_odd);
       !(v == graph.vertexes_end(is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

TEST(AdjacencyListTests, NeighboursTest) {
  AdjacencyList<int, false> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &e);
  graph.add_edge(&b, &e);
  graph.add_edge(&c, &e);
  graph.add_edge(&d, &e);

  std::vector<int> vertexes;
  for (auto v = graph.neighbours_begin(&e); v != graph.neighbours_end(&e);
       ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.neighbours_begin(&e, is_odd);
       !(v == graph.neighbours_end(&e, is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

//--------AdjacencyMatrix tests-----------------

TEST(AdjacencyMatrixTests, VertexTest) {
  AdjacencyMatrix<int, false> graph(3);
  int g;
  ASSERT_TRUE(graph.add_vertex(&a));
  ASSERT_TRUE(graph.add_vertex(&b));
  ASSERT_TRUE(graph.add_vertex(&c));
  auto too_much_vertexes = [&graph, &g]() {
    try {
      graph.add_vertex(&g);
    } catch (std::length_error) {
      return true;
    }
    return false;
  };
  ASSERT_TRUE(too_much_vertexes());
  graph.add_edge(&a, &b);
  ASSERT_TRUE(graph.remove_vertex(&a));
  ASSERT_TRUE(graph.remove_vertex(&c));
  ASSERT_FALSE(test_vertex_remove(graph, &d));
}

TEST(AdjacencyMatrixTests, DirectedNotWeightedEdgeTest) {
  AdjacencyMatrix<int, true> directed_not_weighted(4);

  ASSERT_FALSE(directed_not_weighted.add_edge(&a, &b, 3));
  ASSERT_FALSE(test_add_wrong_unweight_edge(directed_not_weighted, &a, &b));
  directed_not_weighted.add_vertex(&a);
  directed_not_weighted.add_vertex(&b);
  directed_not_weighted.add_vertex(&c);
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &b));
  ASSERT_TRUE(directed_not_weighted.add_edge(&a, &c));
  ASSERT_EQ(directed_not_weighted.edge_weight(&a, &b), 1);
  ASSERT_EQ(directed_not_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(directed_not_weighted, &d, &b));
  ASSERT_TRUE(directed_not_weighted.remove_edge(&a, &b));
  ASSERT_FALSE(directed_not_weighted.remove_edge(&c, &a));
}

TEST(AdjacencyMatrixTests, NotDirectedWeightedEdgeTest) {
  AdjacencyMatrix<int, false, true, int> not_directed_weighted(4);

  ASSERT_FALSE(not_directed_weighted.add_edge(&a, &b));
  ASSERT_FALSE(test_add_wrong_weight_edge(not_directed_weighted, &a, &b, 2));
  not_directed_weighted.add_vertex(&a);
  not_directed_weighted.add_vertex(&b);
  not_directed_weighted.add_vertex(&c);
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &b, 3));
  ASSERT_TRUE(not_directed_weighted.add_edge(&a, &c, 2));
  ASSERT_EQ(not_directed_weighted.edge_weight(&a, &b), 3);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &a), 2);
  ASSERT_EQ(not_directed_weighted.edge_weight(&c, &b), kInf);
  ASSERT_FALSE(test_remove_wrong_edge(not_directed_weighted, &a, &d));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&a, &b));
  ASSERT_TRUE(not_directed_weighted.remove_edge(&c, &a));
}

TEST(AdjacencyMatrixTests, BasicIteratorsTest) {
  AdjacencyMatrix<int, false> graph(4);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);

  std::vector<int> vertexes;
  for (auto v = graph.vertexes_begin(); v != graph.vertexes_end(); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.vertexes_begin(is_odd);
       !(v == graph.vertexes_end(is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

TEST(AdjacencyMatrixTests, NeighboursTest) {
  AdjacencyMatrix<int, false> graph(7);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &e);
  graph.add_edge(&b, &e);
  graph.add_edge(&c, &e);
  graph.add_edge(&d, &e);

  std::vector<int> vertexes;
  for (auto v = graph.neighbours_begin(&e); v != graph.neighbours_end(&e);
       ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(all_vertexes, vertexes);
  vertexes.clear();
  for (auto v = graph.neighbours_begin(&e, is_odd);
       !(v == graph.neighbours_end(&e, is_odd)); ++v) {
    vertexes.push_back(**v);
  }
  std::sort(vertexes.begin(), vertexes.end());
  ASSERT_EQ(odd_vertexes, vertexes);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}