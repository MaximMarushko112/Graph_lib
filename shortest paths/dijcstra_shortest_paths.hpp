#ifndef DIJCSTRA_SHORTEST_PATHS_HPP_
#define DIJCSTRA_SHORTEST_PATHS_HPP_

#include <limits>
#include <set>
#include <unordered_map>
#include <utility>

template <typename Graph>
auto ElogVDijcstraShortestPaths(
    const Graph& graph, const typename Graph::vertex_descriptor& start) {
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>
      shortest_paths;
  const typename Graph::weight kInf =
      std::numeric_limits<typename Graph::weight>::max();
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex)
    shortest_paths[*vertex] = kInf;
  shortest_paths[start] = typename Graph::weight(0);

  std::set<std::pair<typename Graph::weight, typename Graph::vertex_descriptor>>
      available_vertexes;
  available_vertexes.insert({typename Graph::weight(0), start});
  while (!available_vertexes.empty()) {
    typename Graph::vertex_descriptor vertex =
        available_vertexes.begin()->second;
    available_vertexes.erase(available_vertexes.begin());
    for (auto neighbour = graph.neighbours_begin(vertex);
         neighbour != graph.neighbours_end(vertex); ++neighbour) {
      if (shortest_paths[*neighbour] >
          shortest_paths[vertex] + graph.edge_weight(vertex, *neighbour)) {
        available_vertexes.erase({shortest_paths[*neighbour], *neighbour});
        shortest_paths[*neighbour] =
            shortest_paths[vertex] + graph.edge_weight(vertex, *neighbour);
        available_vertexes.insert({shortest_paths[*neighbour], *neighbour});
      }
    }
  }
  return shortest_paths;
}

template <typename Graph>
auto V2DijcstraShortestPaths(const Graph& graph,
                             const typename Graph::vertex_descriptor& start) {
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>
      shortest_paths;
  std::set<typename Graph::vertex_descriptor> unused_vertexes;
  const typename Graph::weight kInf =
      std::numeric_limits<typename Graph::weight>::max();
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex) {
    shortest_paths[*vertex] = kInf;
    unused_vertexes.insert(*vertex);
  }
  shortest_paths[start] = typename Graph::weight(0);

  while (!unused_vertexes.empty()) {
    auto nearest_vertex = unused_vertexes.begin();
    for (auto vertex = unused_vertexes.begin(); vertex != unused_vertexes.end();
         ++vertex) {
      if (shortest_paths[*nearest_vertex] > shortest_paths[*vertex])
        nearest_vertex = vertex;
    }

    for (auto neighbour = graph.neighbours_begin(*nearest_vertex);
         neighbour != graph.neighbours_end(*nearest_vertex); ++neighbour)
      shortest_paths[*neighbour] =
          std::min(shortest_paths[*neighbour],
                   shortest_paths[*nearest_vertex] +
                       graph.edge_weight(*nearest_vertex, *neighbour));
    unused_vertexes.erase(*nearest_vertex);
  }

  return shortest_paths;
}

template <typename Graph, bool IsSparse>
auto DijcstraShortestPaths(const Graph& graph,
                           const typename Graph::vertex_descriptor& start) {
  if (IsSparse) return ElogVDijcstraShortestPaths<Graph>(graph, start);
  return V2DijcstraShortestPaths<Graph>(graph, start);
}

#endif  // DIJCSTRA_SHORTEST_PATHS_HPP_