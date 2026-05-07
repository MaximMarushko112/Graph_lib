#ifndef DIJCSTRA_SHORTEST_PATHS_HPP_
#define DIJCSTRA_SHORTEST_PATHS_HPP_

#include <limits>
#include <set>
#include <unordered_map>
#include <utility>

/**
 * @brief Находит кратчайшие пути от начальной вершины до всех остальных вершин
 * в графе с использованием алгоритма Дейкстры.
 *
 * Этот метод использует структуру данных "множество" для хранения доступных
 * вершин.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены кратчайшие пути.
 * @param start Начальная вершина для поиска кратчайших путей.
 * @return Словарь, содержащий кратчайшие пути от начальной вершины до всех
 * остальных вершин.
 */
template <typename Graph>
auto ElogVDijcstraShortestPaths(
    const Graph& graph, const typename Graph::vertex_descriptor& start) {
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>
      shortest_paths;
  const typename Graph::weight kInf =
      std::numeric_limits<typename Graph::weight>::max();

  // Инициализация расстояний до всех вершин
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex)
    shortest_paths[*vertex] = kInf;
  shortest_paths[start] =
      typename Graph::weight(0);  // Расстояние до начальной вершины равно 0

  std::set<std::pair<typename Graph::weight, typename Graph::vertex_descriptor>>
      available_vertexes;
  available_vertexes.insert(
      {typename Graph::weight(0),
       start});  // Добавляем начальную вершину в множество доступных

  while (!available_vertexes.empty()) {
    typename Graph::vertex_descriptor vertex =
        available_vertexes.begin()
            ->second;  // Извлекаем вершину с минимальным расстоянием
    available_vertexes.erase(available_vertexes.begin());

    // Обновление расстояний до соседей
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
  return shortest_paths;  // Возвращаем найденные кратчайшие пути
}

/**
 * @brief Находит кратчайшие пути от начальной вершины до всех остальных вершин
 * в графе с использованием алгоритма Дейкстры.
 *
 * Этот метод использует структуру данных "множество" для хранения
 * неиспользуемых вершин.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены кратчайшие пути.
 * @param start Начальная вершина для поиска кратчайших путей.
 * @return Словарь, содержащий кратчайшие пути от начальной вершины до всех
 * остальных вершин.
 */
template <typename Graph>
auto V2DijcstraShortestPaths(const Graph& graph,
                             const typename Graph::vertex_descriptor& start) {
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>
      shortest_paths;
  std::set<typename Graph::vertex_descriptor> unused_vertexes;
  const typename Graph::weight kInf =
      std::numeric_limits<typename Graph::weight>::max();

  // Инициализация расстояний до всех вершин
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex) {
    shortest_paths[*vertex] = kInf;
    unused_vertexes.insert(*vertex);
  }
  shortest_paths[start] =
      typename Graph::weight(0);  // Расстояние до начальной вершины равно 0

  while (!unused_vertexes.empty()) {
    auto nearest_vertex = unused_vertexes.begin();
    // Поиск ближайшей неиспользуемой вершины
    for (auto vertex = unused_vertexes.begin(); vertex != unused_vertexes.end();
         ++vertex) {
      if (shortest_paths[*nearest_vertex] > shortest_paths[*vertex])
        nearest_vertex = vertex;
    }

    // Обновление расстояний до соседей
    for (auto neighbour = graph.neighbours_begin(*nearest_vertex);
         neighbour != graph.neighbours_end(*nearest_vertex); ++neighbour)
      shortest_paths[*neighbour] =
          std::min(shortest_paths[*neighbour],
                   shortest_paths[*nearest_vertex] +
                       graph.edge_weight(*nearest_vertex, *neighbour));
    unused_vertexes.erase(
        *nearest_vertex);  // Удаляем ближайшую вершину из множества
  }

  return shortest_paths;  // Возвращаем найденные кратчайшие пути
}

/**
 * @brief Находит кратчайшие пути от начальной вершины до всех остальных вершин
 * в графе с использованием алгоритма Дейкстры.
 *
 * Выбор метода зависит от того, является ли граф разреженным или плотным.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @tparam IsSparse Указывает, является ли граф разреженным.
 * @param graph Граф, в котором будут найдены кратчайшие пути.
 * @param start Начальная вершина для поиска кратчайших путей.
 * @return Словарь, содержащий кратчайшие пути от начальной вершины до всех
 * остальных вершин.
 */
template <typename Graph, bool IsSparse>
auto DijcstraShortestPaths(const Graph& graph,
                           const typename Graph::vertex_descriptor& start) {
  if (IsSparse) return ElogVDijcstraShortestPaths<Graph>(graph, start);
  return V2DijcstraShortestPaths<Graph>(graph, start);
}

#endif  // DIJCSTRA_SHORTEST_PATHS_HPP_