#ifndef DOUBLE_DIJCSTRA_HPP_
#define DOUBLE_DIJCSTRA_HPP_

#include <algorithm>
#include <set>
#include <unordered_map>

template<typename Graph>
auto DoubleDijcstraShortestPath(const Graph& graph, const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish) {
  const typename Graph::weight kInf = std::numeric_limits<typename Graph::weight>::max();
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight> shortest_paths_forward;
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight> shortest_paths_reverse;
  std::set<typename Graph::vertex_descriptor> unused_vertexes_forward;
  std::set<typename Graph::vertex_descriptor> unused_vertexes_reverse;
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end(); ++vertex) {
    shortest_paths_forward[*vertex] = kInf;
    unused_vertexes_forward.insert(*vertex);
    shortest_paths_reverse[*vertex] = kInf;
    unused_vertexes_reverse.insert(*vertex);
  }
  shortest_paths_forward[start] = typename Graph::weight(0);
  shortest_paths_reverse[finish] = typename Graph::weight(0);

  std::set<typename Graph::vertex_descriptor> used_vertexes_forward;
  std::set<typename Graph::vertex_descriptor> used_vertexes_reverse;
  for (;;) {
    auto nearest_vertex_forward = unused_vertexes_forward.begin();
    for (auto vertex = unused_vertexes_forward.begin(); vertex != unused_vertexes_forward.end(); ++vertex) {
      if (shortest_paths_forward[*nearest_vertex_forward] > shortest_paths_forward[*vertex])
        nearest_vertex_forward = vertex;
    }

    for (auto neighbour = graph.neighbours_begin(*nearest_vertex_forward); neighbour != graph.neighbours_end(*nearest_vertex_forward); ++neighbour)
      shortest_paths_forward[*neighbour] = std::min<typename Graph::weight>(shortest_paths_forward[*neighbour], 
          shortest_paths_forward[*nearest_vertex_forward] + graph.edge_weight(*nearest_vertex_forward, *neighbour));
    used_vertexes_forward.insert(*nearest_vertex_forward);
    if (used_vertexes_reverse.find(*nearest_vertex_forward) != used_vertexes_reverse.end())
        break;
    unused_vertexes_forward.erase(*nearest_vertex_forward);

    auto nearest_vertex_reverse = unused_vertexes_reverse.begin();
    for (auto vertex = unused_vertexes_reverse.begin(); vertex != unused_vertexes_reverse.end(); ++vertex) {
      if (shortest_paths_reverse[*nearest_vertex_reverse] > shortest_paths_reverse[*vertex])
        nearest_vertex_reverse = vertex;
    }

    for (auto neighbour = graph.vertexes_begin(); neighbour != graph.vertexes_end(); ++neighbour) {
      if (!graph.edge_in_graph(*neighbour, *nearest_vertex_reverse))
        continue;
      shortest_paths_reverse[*neighbour] = std::min<typename Graph::weight>(shortest_paths_reverse[*neighbour], 
        shortest_paths_reverse[*nearest_vertex_reverse] + graph.edge_weight(*neighbour, *nearest_vertex_reverse));
      }
    
    used_vertexes_reverse.insert(*nearest_vertex_reverse);
    if (used_vertexes_forward.find(*nearest_vertex_reverse) != used_vertexes_forward.end())
        break;
    unused_vertexes_reverse.erase(*nearest_vertex_reverse);
  }
  typename Graph::weight shortest_path = kInf;
  for (auto forward_connect : used_vertexes_forward) {
    for (auto reverse_connect : used_vertexes_reverse) {
      if (graph.edge_weight(forward_connect, reverse_connect) < kInf)
        shortest_path = std::min<typename Graph::weight>(shortest_path, shortest_paths_forward[forward_connect] + 
                  graph.edge_weight(forward_connect, reverse_connect) + shortest_paths_reverse[reverse_connect]);
    }
  }
  return shortest_path;
}

#endif // DOUBLE_DIJCSTRA_HPP_