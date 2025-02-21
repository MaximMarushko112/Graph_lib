#ifndef FLOYD_WARSHALL_SHORTEST_PATHS_HPP_
#define FLOYD_WARSHALL_SHORTEST_PATHS_HPP_

#include <limits>
#include <unordered_map>
#include <utility>

/**
 * @brief Находит кратчайшие пути между всеми парами вершин в графе с
 * использованием алгоритма Флойда-Уоршелла.
 *
 * Алгоритм Флойда-Уоршелла позволяет находить кратчайшие пути между всеми
 * парами вершин в графе, используя динамическое программирование. Он работает
 * для графов с положительными и отрицательными весами рёбер, но не поддерживает
 * графы с отрицательными циклами.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены кратчайшие пути.
 * @return Словарь, содержащий кратчайшие пути между всеми парами вершин.
 */
template <typename Graph>
auto FloydWarshallShortestPaths(const Graph& graph) {
  using vertex_t = typename Graph::vertex_descriptor;
  using weight_t = typename Graph::weight;
  using shortest_paths_helper =
      std::unordered_map<vertex_t, std::unordered_map<vertex_t, weight_t>>;

  const weight_t kInf = std::numeric_limits<weight_t>::max();
  std::pair<shortest_paths_helper, shortest_paths_helper> shortest_paths;
  bool second_helper = true;

  // Инициализация расстояний
  for (auto first = graph.vertexes_begin(); first != graph.vertexes_end();
       ++first) {
    for (auto second = graph.vertexes_begin(); second != graph.vertexes_end();
         ++second) {
      if (first == second)
        shortest_paths.first[*first][*second] = 0;  // Расстояние до самой себя
      else {
        shortest_paths.first[*first][*second] =
            graph.edge_weight(*first, *second);  // Вес ребра
      }
      shortest_paths.second[*first][*second] =
          kInf;  // Инициализация второго помощника
    }
  }

  // Основной цикл алгоритма Флойда-Уоршелла
  for (auto k = graph.vertexes_begin(); k != graph.vertexes_end(); ++k) {
    for (auto first = graph.vertexes_begin(); first != graph.vertexes_end();
         ++first) {
      for (auto second = graph.vertexes_begin(); second != graph.vertexes_end();
           ++second) {
        if (second_helper) {
          if (shortest_paths.first[*first][*k] < kInf &&
              shortest_paths.first[*k][*second] < kInf)
            shortest_paths.second[*first][*second] =
                std::min<weight_t>(shortest_paths.first[*first][*second],
                                   shortest_paths.first[*first][*k] +
                                       shortest_paths.first[*k][*second]);
          else
            shortest_paths.second[*first][*second] =
                shortest_paths.first[*first][*second];
        } else {
          if (shortest_paths.second[*first][*k] < kInf &&
              shortest_paths.second[*k][*second] < kInf)
            shortest_paths.first[*first][*second] =
                std::min<weight_t>(shortest_paths.second[*first][*second],
                                   shortest_paths.second[*first][*k] +
                                       shortest_paths.second[*k][*second]);
          else
            shortest_paths.first[*first][*second] =
                shortest_paths.second[*first][*second];
        }
      }
    }
    second_helper = !second_helper;  // Переключение между помощниками
  }

  // Возвращаем результаты
  if (second_helper) return shortest_paths.first;
  return shortest_paths.second;
}

#endif  // FLOYD_WARSHALL_SHORTEST_PATHS_HPP_