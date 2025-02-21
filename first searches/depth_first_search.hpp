#ifndef DEPTH_FIRST_SEARCH_HPP_
#define DEPTH_FIRST_SEARCH_HPP_

#include <unordered_map>
#include <vector>

#include "colours.hpp"

/**
 * @brief Класс для обработки событий во время обхода графа методом поиска в
 * глубину (DFS).
 *
 * Данный класс предоставляет интерфейс для обработки различных событий,
 * происходящих во время выполнения алгоритма DFS. Пользователь может
 * переопределить методы этого класса для реализации своей логики обработки.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 */
template <typename Graph>
class DFSVisitor {
 public:
  DFSVisitor() = default;

  /**
   * @brief Инициализация вершины перед её посещением.
   *
   * @param vertex Вершина, которую необходимо инициализировать.
   * @param graph Граф, в котором происходит обход.
   */
  void initialize_vertex(const typename Graph::vertex_descriptor& vertex,
                         const Graph& graph) {}

  /**
   * @brief Обработка события начала исследования вершины.
   *
   * @param vertex Вершина, которую необходимо начать исследовать.
   * @param graph Граф, в котором происходит обход.
   */
  void start_vertex(const typename Graph::vertex_descriptor& vertex,
                    const Graph& graph) {}

  /**
   * @brief Обработка события обнаружения вершины.
   *
   * @param vertex Вершина, которая была обнаружена.
   * @param graph Граф, в котором происходит обход.
   */
  void discover_vertex(const typename Graph::vertex_descriptor& vertex,
                       const Graph& graph) {}

  /**
   * @brief Обработка события исследования вершины.
   *
   * @param vertex Вершина, которая исследуется.
   * @param graph Граф, в котором происходит обход.
   */
  void examine_vertex(const typename Graph::vertex_descriptor& vertex,
                      const Graph& graph) {}

  /**
   * @brief Обработка события завершения исследования вершины.
   *
   * @param vertex Вершина, исследование которой завершено.
   * @param graph Граф, в котором происходит обход.
   */
  void finish_vertex(const typename Graph::vertex_descriptor& vertex,
                     const Graph& graph) {}

  /**
   * @brief Обработка события исследования ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void examine_edge(const typename Graph::vertex_descriptor& start,
                    const typename Graph::vertex_descriptor& finish,
                    const Graph& graph) {}

  /**
   * @brief Обработка события обнаружения дерева.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void tree_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {}

  /**
   * @brief Обработка события обнаружения обратного ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void back_edge(const typename Graph::vertex_descriptor& start,
                 const typename Graph::vertex_descriptor& finish,
                 const Graph& graph) {}

  /**
   * @brief Обработка события обнаружения прямого или перекрестного ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void forward_or_cross_edge(const typename Graph::vertex_descriptor& start,
                             const typename Graph::vertex_descriptor& finish,
                             const Graph& graph) {}

  /**
   * @brief Обработка события завершения исследования ребра.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void finish_edge(const typename Graph::vertex_descriptor& start,
                   const typename Graph::vertex_descriptor& finish,
                   const Graph& graph) {}
};

/**
 * @brief Выполняет поиск в глубину (DFS) по графу.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @tparam Visitor Тип посетителя, который обрабатывает события DFS.
 *
 * @param g Граф, по которому будет выполнен поиск.
 * @param root Корневая вершина для начала поиска.
 * @param visitor Объект посетителя для обработки событий.
 * @param colours Словарь, хранящий цвета вершин.
 */
template <typename Graph, typename Visitor>
void DepthFirstSearch(
    const Graph& g, typename Graph::vertex_descriptor root, Visitor visitor,
    std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  // Инициализация всех вершин
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    visitor.initialize_vertex(*vertex, g);
    colours[*vertex] = Colour::White;  // Устанавливаем цвет вершины в белый
  }

  visitor.start_vertex(root,
                       g);  // Обработка начала исследования корневой вершины
  DepthFirstVisit(g, root, visitor, colours);  // Рекурсивный обход

  // Обработка оставшихся белых вершин
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    if (colours[*vertex] == Colour::White) {
      visitor.start_vertex(*vertex, g);
      DepthFirstVisit(g, *vertex, visitor, colours);
    }
  }
}

/**
 * @brief Рекурсивная функция для выполнения обхода в глубину (DFS).
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @tparam Visitor Тип посетителя, который обрабатывает события DFS.
 *
 * @param g Граф, по которому будет выполнен поиск.
 * @param root Корневая вершина для начала обхода.
 * @param visitor Объект посетителя для обработки событий.
 * @param colours Словарь, хранящий цвета вершин.
 */
template <typename Graph, typename Visitor>
void DepthFirstVisit(
    const Graph& g, typename Graph::vertex_descriptor root, Visitor& visitor,
    std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  colours[root] = Colour::Gray;  // Устанавливаем цвет вершины в серый
  visitor.discover_vertex(root, g);  // Обработка обнаружения вершины

  // Исследуем соседей текущей вершины
  for (auto neighbour = g.neighbours_begin(root);
       neighbour != g.neighbours_end(root); ++neighbour) {
    visitor.examine_edge(root, *neighbour, g);  // Исследуем ребро
    Colour neighbour_colour = colours[*neighbour];  // Получаем цвет соседа
    if (neighbour_colour == Colour::White) {
      visitor.tree_edge(root, *neighbour, g);  // Обрабатываем дерево
      DepthFirstVisit(g, *neighbour, visitor,
                      colours);  // Рекурсивный вызов для соседа
    } else if (neighbour_colour == Colour::Gray) {
      visitor.back_edge(root, *neighbour, g);  // Обрабатываем обратное ребро
    } else {
      visitor.forward_or_cross_edge(
          root, *neighbour, g);  // Обрабатываем прямое или перекрестное ребро
    }
    visitor.finish_edge(root, *neighbour, g);  // Завершение исследования ребра
  }
  colours[root] = Colour::Black;  // Устанавливаем цвет вершины в черный
  visitor.finish_vertex(root, g);  // Завершение исследования вершины
}

#endif  // DEPTH_FIRST_SEARCH_HPP_