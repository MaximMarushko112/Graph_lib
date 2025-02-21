#ifndef FLOYD_WARSHALL_HPP_
#define FLOYD_WARSHALL_HPP_

#include <iostream>
#include <unordered_map>
#include <utility>

template <typename Graph>
auto FloydWarshallShortestPaths(const Graph& graph) {
  using vertex_t = typename Graph::vertex_descriptor;
  using weight_t = typename Graph::weight;
  using shortest_paths_helper = std::unordered_map<vertex_t, std::unordered_map<vertex_t, weight_t>>;
  const weight_t kInf = std::numeric_limits<weight_t>::max();
  std::pair<shortest_paths_helper, shortest_paths_helper>  shortest_paths;
  bool second_helper = true;

  for (auto first = graph.vertexes_begin(); first != graph.vertexes_end(); ++first) {
    for (auto second = graph.vertexes_begin(); second != graph.vertexes_end(); ++second) {
      if (first == second)
        shortest_paths.first[*first][*second] = 0;
      else {
        shortest_paths.first[*first][*second] = graph.edge_weight(*first, *second);
      }
      shortest_paths.second[*first][*second] = kInf;
    }
  }

  for (auto k = graph.vertexes_begin(); k != graph.vertexes_end(); ++k) {
    for (auto first = graph.vertexes_begin(); first != graph.vertexes_end(); ++first) {
      for (auto second = graph.vertexes_begin(); second != graph.vertexes_end(); ++second) {
        if (second_helper) {
          if (shortest_paths.first[*first][*k] < kInf && shortest_paths.first[*k][*second] < kInf)
            shortest_paths.second[*first][*second] = std::min<weight_t>(shortest_paths.first[*first][*second], 
                                                              shortest_paths.first[*first][*k] + shortest_paths.first[*k][*second]);
          else 
            shortest_paths.second[*first][*second] = shortest_paths.first[*first][*second];
        } 
        else {
          if (shortest_paths.second[*first][*k] < kInf && shortest_paths.second[*k][*second] < kInf)
            shortest_paths.first[*first][*second] = std::min<weight_t>(shortest_paths.second[*first][*second], 
                                                             shortest_paths.second[*first][*k] + shortest_paths.second[*k][*second]);
          else
            shortest_paths.first[*first][*second] = shortest_paths.second[*first][*second];
        }
      }
    }
    second_helper = !second_helper;
  } 
  if (second_helper)
    return shortest_paths.first;
  return shortest_paths.second;
}

#endif // FLOYD_WARSHALL_HPP_