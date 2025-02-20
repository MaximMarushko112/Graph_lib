#ifndef FORD_BELLMAN_HPP_
#define FORD_BELLMAN_HPP_

#include <algorithm>
#include <limits>
#include <vector>

#include "../first searches/breadth_first_search.hpp"

template<typename Graph>
class negative_cycle_visitor : public BFSVisitor<Graph> {
 public:
  negative_cycle_visitor(std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>* const & shortest_paths) : 
    shortest_paths_(shortest_paths) {}
  
  void discover_vertex  (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    (*shortest_paths_)[vertex] = std::numeric_limits<typename Graph::weight>::min();
  }

 private:
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>* shortest_paths_;
};

template<typename Graph>
auto ford_bellman(const Graph& graph, const typename Graph::vertex_descriptor& start) {
  const typename Graph::weight kInf = std::numeric_limits<typename Graph::weight>::max();
  const typename Graph::weight kNegInf = std::numeric_limits<typename Graph::weight>::min();
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight> shortest_paths;
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end(); ++vertex) 
    shortest_paths[*vertex] = kInf;
  shortest_paths[start] = typename Graph::weight(0);
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor> parents;

  typename Graph::vertex_descriptor negative_cycle_vertex;
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end(); ++vertex) {
    negative_cycle_vertex = nullptr;
    for (auto edge = graph.edges_begin(); edge != graph.edges_end(); ++edge) {
      if (shortest_paths[edge->first] < kInf) {
        if (shortest_paths[edge->second] > shortest_paths[edge->first] + graph.edge_weight(edge->first, edge->second)) {
          shortest_paths[edge->second] = std::max<typename Graph::weight>(kNegInf, 
                                                            shortest_paths[edge->first] + graph.edge_weight(edge->first, edge->second));
          parents[edge->second] = edge->first;
          negative_cycle_vertex = edge->second;
        }
      }
    }
  }
  if (negative_cycle_vertex != nullptr) {
    std::vector<typename Graph::vertex_descriptor> cycle;
    for (auto i = graph.vertexes_begin(); i != graph.vertexes_end(); ++i) {
      cycle.push_back(negative_cycle_vertex);
      negative_cycle_vertex = parents[negative_cycle_vertex];
    }
    std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
    breadth_first_search(graph, cycle, negative_cycle_visitor<Graph>(&shortest_paths), colours);
  }

  return shortest_paths;
}

#endif //FORD_BELLMAN_HPP_