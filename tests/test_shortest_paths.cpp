#include <gtest/gtest.h>
#include <unordered_map>

#include "../include/graph types/edge_list.hpp"
#include "../include/graph types/adjacency_list.hpp"
#include "../include/graph types/adjacency_matrix.hpp"
#include "../include/shortest paths/dijcstra_shortest_paths.hpp"
#include "../include/shortest paths/double_dijcstra_shortest_path.hpp"
#include "../include/shortest paths/ford_bellman_shortest_paths.hpp"
#include "../include/shortest paths/floyd_warshall_shortest_paths.hpp"

int a = 1, b = 2, c = 3, d = 4, e = 5, f = 6, g = 7, h = 8, i = 9, j = 10;
const std::size_t kInf = std::numeric_limits<std::size_t>::max();
const std::size_t kNegInf = std::numeric_limits<std::size_t>::min();
std::unordered_map<int*, std::size_t> from_a{{&a, 0}, {&b, 10}, {&c, 12}, 
  {&d, 18}, {&e, 15}, {&f, 13}, {&g, 19}, {&h, 26}, {&i, 28}, {&j, kInf}};

std::unordered_map<int*, std::unordered_map<int*, std::size_t>> fwtest{
  {&a, {{&a, 0}, {&b, 1}, {&c, 2}, {&d, kInf}, {&e, kInf}}},
  {&b, {{&a, 1}, {&b, 0}, {&c, 3}, {&d, kInf}, {&e, kInf}}},
  {&c, {{&a, 2}, {&b, 3}, {&c, 0}, {&d, kInf}, {&e, kInf}}},
  {&d, {{&a, kInf}, {&b, kInf}, {&c, kInf}, {&d, 0}, {&e, 5}}},
  {&e, {{&a, kInf}, {&b, kInf}, {&c, kInf}, {&d, 5}, {&e, 0}}},
};

//--------------Dijcstra tests-----------------

TEST(DijcstraTests, EdgeListTest) {
  EdgeList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = DijcstraShortestPaths<EdgeList<int, false, true, std::size_t>, true>(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

TEST(DijcstraTests, AdjacencyListTest) {
  AdjacencyList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = DijcstraShortestPaths<AdjacencyList<int, false, true, std::size_t>, true>(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

TEST(DijcstraTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false, true, std::size_t> graph(10);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = DijcstraShortestPaths<AdjacencyMatrix<int, false, true, std::size_t>, false>(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

//--------FordBellman tests--------------------

TEST(FordBellmanTests, EdgeListTest) {
  EdgeList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

TEST(FordBellmanTests, AdjacencyListTest) {
  AdjacencyList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

TEST(FordBellmanTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false, true, std::size_t> graph(10);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) 
    if (path.second != from_a[path.first]) test = false;
  ASSERT_TRUE(test);
}

TEST(FordBellmanTests, NegativeCycleEdgeListTest) {
  EdgeList<int, false, true, int> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, -10);
  graph.add_edge(&a, &c, -12);
  graph.add_edge(&b, &d, -8);
  graph.add_edge(&c, &e, -3);
  graph.add_edge(&e, &d, -7);
  graph.add_edge(&c, &f, -1);
  graph.add_edge(&e, &f, -3);
  graph.add_edge(&d, &g, -5);
  graph.add_edge(&d, &h, -8);
  graph.add_edge(&f, &g, -6);
  graph.add_edge(&g, &h, -9);
  graph.add_edge(&g, &i, -11);
  graph.add_edge(&h, &i, -2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) {
    if (path.first != &j) 
      if (path.second != std::numeric_limits<int>::min()) test = false;
    else
      if (path.second != std::numeric_limits<int>::min()) test = false;
  }
  ASSERT_TRUE(test);
}

TEST(FordBellmanTests, NegativeCycleAdjacencyListTest) {
  AdjacencyList<int, false, true, int> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, -10);
  graph.add_edge(&a, &c, -12);
  graph.add_edge(&b, &d, -8);
  graph.add_edge(&c, &e, -3);
  graph.add_edge(&e, &d, -7);
  graph.add_edge(&c, &f, -1);
  graph.add_edge(&e, &f, -3);
  graph.add_edge(&d, &g, -5);
  graph.add_edge(&d, &h, -8);
  graph.add_edge(&f, &g, -6);
  graph.add_edge(&g, &h, -9);
  graph.add_edge(&g, &i, -11);
  graph.add_edge(&h, &i, -2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) {
    if (path.first != &j) 
      if (path.second != std::numeric_limits<int>::min()) test = false;
    else
      if (path.second != std::numeric_limits<int>::min()) test = false;
  }
  ASSERT_TRUE(test);
}

TEST(FordBellmanTests, NegativeCycleAdjacencyMatrixTest) {
  AdjacencyMatrix<int, false, true, int> graph(10);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, -10);
  graph.add_edge(&a, &c, -12);
  graph.add_edge(&b, &d, -8);
  graph.add_edge(&c, &e, -3);
  graph.add_edge(&e, &d, -7);
  graph.add_edge(&c, &f, -1);
  graph.add_edge(&e, &f, -3);
  graph.add_edge(&d, &g, -5);
  graph.add_edge(&d, &h, -8);
  graph.add_edge(&f, &g, -6);
  graph.add_edge(&g, &h, -9);
  graph.add_edge(&g, &i, -11);
  graph.add_edge(&h, &i, -2);
  
  bool test = true;
  auto res = FordBellmanShortestPaths(graph, &a);
  for (auto path : res) {
    if (path.first != &j) 
      if (path.second != std::numeric_limits<int>::min()) test = false;
    else
      if (path.second != std::numeric_limits<int>::min()) test = false;
  }
  ASSERT_TRUE(test);
}

//--------DoubleDijcstra tests-----------------

TEST(DoubleDijcstraTests, EdgeListTest) {
  EdgeList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  ASSERT_EQ(28, DoubleDijcstraShortestPath(graph, &a, &i));
  ASSERT_EQ(kInf, DoubleDijcstraShortestPath(graph, &c, &j));
}

TEST(DoubleDijcstraTests, AdjacencyListTest) {
  AdjacencyList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  ASSERT_EQ(28, DoubleDijcstraShortestPath(graph, &a, &i));
  ASSERT_EQ(kInf, DoubleDijcstraShortestPath(graph, &c, &j));
}

TEST(DoubleDijcstraTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false, true, std::size_t> graph(10);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_vertex(&f);
  graph.add_vertex(&g);
  graph.add_vertex(&h);
  graph.add_vertex(&i);
  graph.add_vertex(&j);
  graph.add_edge(&a, &b, 10);
  graph.add_edge(&a, &c, 12);
  graph.add_edge(&b, &d, 8);
  graph.add_edge(&c, &e, 3);
  graph.add_edge(&e, &d, 7);
  graph.add_edge(&c, &f, 1);
  graph.add_edge(&e, &f, 3);
  graph.add_edge(&d, &g, 5);
  graph.add_edge(&d, &h, 8);
  graph.add_edge(&f, &g, 6);
  graph.add_edge(&g, &h, 9);
  graph.add_edge(&g, &i, 11);
  graph.add_edge(&h, &i, 2);
  
  ASSERT_EQ(28, DoubleDijcstraShortestPath(graph, &a, &i));
  ASSERT_EQ(kInf, DoubleDijcstraShortestPath(graph, &c, &j));
}

//--------FloydWarshall tests--------------------

TEST(FloydWarshallTests, EdgeListTest) {
  EdgeList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &b, 1);
  graph.add_edge(&a, &c, 2);
  graph.add_edge(&b, &c, 4);
  graph.add_edge(&d, &e, 5);
  
  bool test = true;
  auto res = FloydWarshallShortestPaths(graph);
  for (auto paths : res) {
    for (auto path : paths.second) {
      if (res[paths.first][path.first] != fwtest[paths.first][path.first]) 
        test = false;
    }
  }
  ASSERT_TRUE(test);
}

TEST(FloydWarshallTests, AdjacencyListTest) {
  AdjacencyList<int, false, true, std::size_t> graph;
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &b, 1);
  graph.add_edge(&a, &c, 2);
  graph.add_edge(&b, &c, 4);
  graph.add_edge(&d, &e, 5);
  
  bool test = true;
  auto res = FloydWarshallShortestPaths(graph);
  for (auto paths : res) {
    for (auto path : paths.second) {
      if (res[paths.first][path.first] != fwtest[paths.first][path.first]) 
        test = false;
    }
  }
  ASSERT_TRUE(test);
}

TEST(FloydWarshallTests, AdjacencyMatrixTest) {
  AdjacencyMatrix<int, false, true, std::size_t> graph(5);
  graph.add_vertex(&a);
  graph.add_vertex(&b);
  graph.add_vertex(&c);
  graph.add_vertex(&d);
  graph.add_vertex(&e);
  graph.add_edge(&a, &b, 1);
  graph.add_edge(&a, &c, 2);
  graph.add_edge(&b, &c, 4);
  graph.add_edge(&d, &e, 5);
  
  bool test = true;
  auto res = FloydWarshallShortestPaths(graph);
  for (auto paths : res) {
    for (auto path : paths.second) {
      if (res[paths.first][path.first] != fwtest[paths.first][path.first]) 
        test = false;
    }
  }
  ASSERT_TRUE(test);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}