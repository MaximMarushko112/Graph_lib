#ifndef FORD_BELLMAN_SHORTEST_PATHS_HPP_
#define FORD_BELLMAN_SHORTEST_PATHS_HPP_

#include <algorithm>
#include <limits>
#include <vector>

#include "../first searches/breadth_first_search.hpp"

/**
 * @brief Класс для обработки отрицательных циклов в графе во время обхода
 * методом поиска в ширину (BFS).
 *
 * Данный класс наследует от BFSVisitor и реализует логику для обработки вершин,
 * находящихся в отрицательных циклах, устанавливая их расстояние в минус
 * бесконечность.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 */
template <typename Graph>
class NegativeCycleVisitor : public BFSVisitor<Graph> {
 public:
  /**
   * @brief Конструктор класса NegativeCycleVisitor.
   *
   * @param shortest_paths Указатель на словарь кратчайших путей.
   */
  NegativeCycleVisitor(
      std::unordered_map<typename Graph::vertex_descriptor,
                         typename Graph::weight>* const& shortest_paths)
      : shortest_paths_(shortest_paths) {}

  /**
   * @brief Обработка события обнаружения вершины.
   *
   * @param vertex Вершина, которая была обнаружена.
   * @param graph Граф, в котором происходит обход.
   */
  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
                       const Graph& graph) {
    (*shortest_paths_)[vertex] = std::numeric_limits<typename Graph::weight>::
        min();  // Устанавливаем расстояние в минус бесконечность
  }

 private:
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>*
      shortest_paths_;  ///< Указатель на словарь кратчайших путей.
};

/**
 * @brief Находит кратчайшие пути в графе с использованием алгоритма
 * Форда-Беллмана.
 *
 * Алгоритм Форда-Беллмана позволяет находить кратчайшие пути от одной вершины
 * до всех остальных в графе, включая графы с отрицательными весами рёбер. Если
 * в графе есть отрицательный цикл, алгоритм помечает все вершины, находящиеся в
 * этом цикле, расстоянием в минус бесконечность.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены кратчайшие пути.
 * @param start Начальная вершина для поиска кратчайших путей.
 * @return Словарь, содержащий кратчайшие пути от начальной вершины до всех
 * остальных вершин.
 */
template <typename Graph>
auto FordBellmanShortestPaths(const Graph& graph,
                              const typename Graph::vertex_descriptor& start) {
  const typename Graph::weight kInf =
      std::numeric_limits<typename Graph::weight>::max();
  const typename Graph::weight kNegInf =
      std::numeric_limits<typename Graph::weight>::min();

  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::weight>
      shortest_paths;

  // Инициализация расстояний
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex)
    shortest_paths[*vertex] = kInf;

  shortest_paths[start] = typename Graph::weight(0);
  std::unordered_map<typename Graph::vertex_descriptor,
                     typename Graph::vertex_descriptor>
      parents;

  typename Graph::vertex_descriptor negative_cycle_vertex;

  // Основной цикл алгоритма Форда-Беллмана
  for (auto vertex = graph.vertexes_begin(); vertex != graph.vertexes_end();
       ++vertex) {
    negative_cycle_vertex = nullptr;
    for (auto edge = graph.edges_begin(); edge != graph.edges_end(); ++edge) {
      if (shortest_paths[edge->first] < kInf) {
        if (shortest_paths[edge->second] >
            shortest_paths[edge->first] +
                graph.edge_weight(edge->first, edge->second)) {
          shortest_paths[edge->second] = std::max<typename Graph::weight>(
              kNegInf, shortest_paths[edge->first] +
                           graph.edge_weight(edge->first, edge->second));
          parents[edge->second] = edge->first;
          negative_cycle_vertex = edge->second;
        }
      }
    }
  }

  // Обработка отрицательного цикла
  if (negative_cycle_vertex != nullptr) {
    std::vector<typename Graph::vertex_descriptor> cycle;
    for (auto i = graph.vertexes_begin(); i != graph.vertexes_end(); ++i) {
      cycle.push_back(negative_cycle_vertex);
      negative_cycle_vertex = parents[negative_cycle_vertex];
    }
    std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
    BreadthFirstSearch(graph, cycle,
                       NegativeCycleVisitor<Graph>(&shortest_paths), colours);
  }

  return shortest_paths;  // Возвращаем кратчайшие пути
}

#endif  // FORD_BELLMAN_SHORTEST_PATHS_HPP_