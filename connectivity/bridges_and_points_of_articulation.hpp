#ifndef BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_
#define BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "../first searches/depth_first_search.hpp"
#include "../utils/edge_hash.hpp"

/**
 * @brief Класс для обработки мостов в графе во время обхода методом поиска в
 * глубину (DFS).
 *
 * Данный класс наследует от DFSVisitor и реализует логику для нахождения мостов
 * в графе. Мостом называется ребро, удаление которого увеличивает количество
 * компонент связности графа.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 */
template <typename Graph>
class BridgeVisitor : public DFSVisitor<Graph> {
 public:
  /**
   * @brief Конструктор класса BridgeVisitor.
   *
   * @param bridges Указатель на множество мостов, которые будут найдены.
   */
  BridgeVisitor(std::unordered_set<std::pair<typename Graph::vertex_descriptor,
                                             typename Graph::vertex_descriptor>,
                                   typename Graph::edge_hash>* const& bridges)
      : bridges_(bridges), timer_(0) {}

  /**
   * @brief Обработка события обнаружения вершины.
   *
   * @param vertex Вершина, которая была обнаружена.
   * @param graph Граф, в котором происходит обход.
   */
  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
                       const Graph& graph) {
    ++timer_;
    tin_[vertex] = timer_;
    fup_[vertex] = timer_;
  }

  /**
   * @brief Обработка события завершения исследования вершины.
   *
   * @param vertex Вершина, исследование которой завершено.
   * @param graph Граф, в котором происходит обход.
   */
  void finish_vertex(const typename Graph::vertex_descriptor& vertex,
                     const Graph& graph) {
    std::size_t tin_v = tin_[vertex];
    std::size_t fup_v = fup_[vertex];

    for (auto tree_neighbour = tree_edges_[vertex].begin();
         tree_neighbour != tree_edges_[vertex].end(); ++tree_neighbour)
      fup_v = std::min<std::size_t>(fup_v, fup_[*tree_neighbour]);
    fup_[vertex] = fup_v;

    for (auto neighbour = graph.neighbours_begin(vertex);
         neighbour != graph.neighbours_end(vertex); ++neighbour) {
      if (fup_[*neighbour] > tin_v) bridges_->insert({vertex, *neighbour});
    }
  }

  /**
   * @brief Обработка события обнаружения дерева.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void tree_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {
    parents_[finish] = start;
    tree_edges_[start].insert(finish);
  }

  /**
   * @brief Обработка события обнаружения обратного ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void back_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {
    if (finish != parents_[start])
      fup_[start] = std::min<std::size_t>(fup_[start], tin_[finish]);
  }

 private:
  std::size_t timer_;  ///< Таймер для отслеживания времени обнаружения вершин.
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t>
      tin_;  ///< Время обнаружения вершин.
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t>
      fup_;  ///< Минимальное время достижения.
  std::unordered_map<typename Graph::vertex_descriptor,
                     typename Graph::vertex_descriptor>
      parents_;  ///< Родители вершин в дереве обхода.
  std::unordered_map<typename Graph::vertex_descriptor,
                     std::unordered_set<typename Graph::vertex_descriptor>>
      tree_edges_;  ///< Деревья обхода.
  std::unordered_set<std::pair<typename Graph::vertex_descriptor,
                               typename Graph::vertex_descriptor>,
                     typename Graph::edge_hash>*
      bridges_;  ///< Множество найденных мостов.
};

/**
 * @brief Находит мосты в графе.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены мосты.
 * @return Множество найденных мостов в графе.
 */
template <typename Graph>
auto FindBridges(const Graph& graph) {
  std::unordered_set<std::pair<typename Graph::vertex_descriptor,
                               typename Graph::vertex_descriptor>,
                     typename Graph::edge_hash>
      bridges;
  std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
  DepthFirstSearch(graph, *graph.vertexes_begin(),
                   BridgeVisitor<Graph>(&bridges), colours);
  return bridges;  // Возвращаем найденные мосты
}

/**
 * @brief Класс для обработки точек сочленения в графе во время обхода методом
 * поиска в глубину (DFS).
 *
 * Данный класс наследует от DFSVisitor и реализует логику для нахождения точек
 * сочленения в графе. Точка сочленения — это вершина, удаление которой
 * увеличивает количество компонент связности графа.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 */
template <typename Graph>
class PointsOfArticulationVisitor : public DFSVisitor<Graph> {
 public:
  /**
   * @brief Конструктор класса PointsOfArticulationVisitor.
   *
   * @param points_of_articulation Указатель на множество точек сочленения,
   * которые будут найдены.
   */
  PointsOfArticulationVisitor(
      std::unordered_set<typename Graph::vertex_descriptor>* const&
          points_of_articulation)
      : points_of_articulation_(points_of_articulation),
        timer_(0),
        start_recursive_calls_count_(0) {}

  /**
   * @brief Обработка события начала исследования вершины.
   *
   * @param vertex Вершина, которую необходимо начать исследовать.
   * @param graph Граф, в котором происходит обход.
   */
  void start_vertex(const typename Graph::vertex_descriptor& vertex,
                    const Graph& graph) {
    start_ = vertex;  // Запоминаем начальную вершину
    start_recursive_calls_count_ = 0;
  }

  /**
   * @brief Обработка события обнаружения вершины.
   *
   * @param vertex Вершина, которая была обнаружена.
   * @param graph Граф, в котором происходит обход.
   */
  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
                       const Graph& graph) {
    ++timer_;
    tin_[vertex] = timer_;
    fup_[vertex] = timer_;
  }

  /**
   * @brief Обработка события завершения исследования вершины.
   *
   * @param vertex Вершина, исследование которой завершено.
   * @param graph Граф, в котором происходит обход.
   */
  void finish_vertex(const typename Graph::vertex_descriptor& vertex,
                     const Graph& graph) {
    std::size_t tin_v = tin_[vertex];
    std::size_t fup_v = fup_[vertex];

    if (vertex == start_) return;  // Пропускаем начальную вершину

    for (auto tree_neighbour = tree_edges_[vertex].begin();
         tree_neighbour != tree_edges_[vertex].end(); ++tree_neighbour)
      fup_v = std::min<std::size_t>(fup_v, fup_[*tree_neighbour]);
    fup_[vertex] = fup_v;

    bool is_point_of_articulation = true;
    for (auto neighbour = graph.neighbours_begin(vertex);
         neighbour != graph.neighbours_end(vertex); ++neighbour) {
      if (*neighbour != parents_[vertex] && fup_[*neighbour] < tin_v) {
        is_point_of_articulation =
            false;  // Если сосед не является точкой сочленения
        break;
      }
    }
    if (is_point_of_articulation)
      points_of_articulation_->insert(vertex);  // Добавляем точку сочленения
  }

  /**
   * @brief Обработка события обнаружения дерева.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void tree_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {
    parents_[finish] = start;
    tree_edges_[start].insert(finish);
    if (start == start_) {
      ++start_recursive_calls_count_;
      if (start_recursive_calls_count_ > 1) {
        points_of_articulation_->insert(
            start);  // Если начальная вершина имеет более одного дочернего узла
      }
    }
  }

  /**
   * @brief Обработка события обнаружения обратного ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void back_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {
    if (finish != parents_[start])
      fup_[start] = std::min<std::size_t>(fup_[start], tin_[finish]);
  }

 private:
  std::size_t timer_;  ///< Таймер для отслеживания времени обнаружения вершин.
  typename Graph::vertex_descriptor start_;  ///< Начальная вершина для обхода.
  std::size_t start_recursive_calls_count_;  ///< Счетчик рекурсивных вызовов
                                             ///< для начальной вершины.
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t>
      tin_;  ///< Время обнаружения вершин.
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t>
      fup_;  ///< Вспомогательная функция для поиска точек сочленения.
  std::unordered_map<typename Graph::vertex_descriptor,
                     typename Graph::vertex_descriptor>
      parents_;  ///< Родители вершин в дереве обхода.
  std::unordered_map<typename Graph::vertex_descriptor,
                     std::unordered_set<typename Graph::vertex_descriptor>>
      tree_edges_;  ///< Рёбра дерева обхода.
  std::unordered_set<typename Graph::vertex_descriptor>*
      points_of_articulation_;  ///< Множество найденных точек сочленения.
};

/**
 * @brief Находит точки сочленения в графе.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @param graph Граф, в котором будут найдены точки сочленения.
 * @return Множество найденных точек сочленения в графе.
 */
template <typename Graph>
auto FindPointsOfArticulation(const Graph& graph) {
  std::unordered_set<typename Graph::vertex_descriptor> points_of_articulation;
  std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
  DepthFirstSearch(graph, *graph.vertexes_begin(),
                   PointsOfArticulationVisitor<Graph>(&points_of_articulation),
                   colours);
  return points_of_articulation;  // Возвращаем найденные точки сочленения
}

#endif  // BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_