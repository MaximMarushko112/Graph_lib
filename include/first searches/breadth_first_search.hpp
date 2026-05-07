#ifndef BREADTH_FIRST_SEARCH_HPP_
#define BREADTH_FIRST_SEARCH_HPP_

#include <queue>
#include <unordered_map>
#include <vector>

#include "colours.hpp"

/**
 * @brief Класс для обработки событий во время обхода графа методом поиска в
 * ширину (BFS).
 *
 * Данный класс предоставляет интерфейс для обработки различных событий,
 * происходящих во время выполнения алгоритма BFS. Пользователь может
 * переопределить методы этого класса для реализации своей логики обработки.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 */
template <typename Graph>
class BFSVisitor {
 public:
  BFSVisitor() = default;

  /**
   * @brief Инициализация вершины перед её посещением.
   *
   * @param vertex Вершина, которую необходимо инициализировать.
   * @param graph Граф, в котором происходит обход.
   */
  void initialize_vertex(const typename Graph::vertex_descriptor& vertex,
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
   * @brief Обработка события обнаружения не-дерева.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void non_tree_edge(const typename Graph::vertex_descriptor& start,
                     const typename Graph::vertex_descriptor& finish,
                     const Graph& graph) {}

  /**
   * @brief Обработка события, когда целевая вершина серого цвета.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void gray_target(const typename Graph::vertex_descriptor& start,
                   const typename Graph::vertex_descriptor& finish,
                   const Graph& graph) {}

  /**
   * @brief Обработка события, когда целевая вершина черного цвета.
   *
   * @param start Вершина, из которой начинается ребро.
   * @param finish Вершина, в которую ведет ребро.
   * @param graph Граф, в котором происходит обход.
   */
  void black_target(const typename Graph::vertex_descriptor& start,
                    const typename Graph::vertex_descriptor& finish,
                    const Graph& graph) {}
};

/** 
 * @brief Выполняет поиск в ширину (BFS) по графу.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @tparam Visitor Тип посетителя, который обрабатывает события BFS.
 *
 * @param g Граф, по которому будет выполнен поиск.
 * @param roots Вектор корневых вершин для начала поиска.
 * @param visitor Объект посетителя для обработки событий.
 * @param colours Словарь, хранящий цвета вершин.
 */
template <typename Graph, typename Visitor>
void BreadthFirstSearch(
    const Graph& g, std::vector<typename Graph::vertex_descriptor> const& roots,
    Visitor visitor,
    std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  std::queue<typename Graph::vertex_descriptor> queue;

  // Инициализация всех вершин
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    visitor.initialize_vertex(*vertex, g);
    colours[*vertex] = Colour::White;  // Устанавливаем цвет вершины в белый
  }

  // Обработка корневых вершин
  for (auto root = roots.begin(); root != roots.end(); ++root) {
    colours[*root] =
        Colour::Gray;  // Устанавливаем цвет корневой вершины в серый
    visitor.discover_vertex(*root, g);
    queue.push(*root);  // Добавляем корневую вершину в очередь
  }

  // Основной цикл BFS
  while (!queue.empty()) {
    auto vertex = queue.front();
    queue.pop();
    visitor.examine_vertex(vertex, g);  // Исследуем текущую вершину

    // Исследуем соседей текущей вершины
    for (auto neighbour = g.neighbours_begin(vertex);
         neighbour != g.neighbours_end(vertex); ++neighbour) {
      visitor.examine_edge(vertex, *neighbour, g);  // Исследуем ребро
      Colour neighbour_colour = colours[*neighbour];  // Получаем цвет соседа
      if (neighbour_colour == Colour::White) {
        visitor.tree_edge(vertex, *neighbour, g);  // Обрабатываем дерево
        colours[*neighbour] =
            Colour::Gray;  // Устанавливаем цвет соседа в серый
        visitor.discover_vertex(*neighbour,
                                g);  // Обрабатываем обнаружение соседа
        queue.push(*neighbour);  // Добавляем соседа в очередь
      } else {
        visitor.non_tree_edge(vertex, *neighbour, g);  // Обрабатываем не-дерево
        if (neighbour_colour == Colour::Gray)
          visitor.gray_target(vertex, *neighbour,
                              g);  // Обрабатываем серую цель
        else
          visitor.black_target(vertex, *neighbour,
                               g);  // Обрабатываем черную цель
      }
    }

    colours[vertex] =
        Colour::Black;  // Устанавливаем цвет текущей вершины в черный
    visitor.finish_vertex(vertex, g);  // Завершаем исследование вершины
  }
}

/**
 * @brief Выполняет поиск в ширину (BFS) по графу, начиная с одной корневой
 * вершины.
 *
 * @tparam Graph Тип графа, который будет обрабатываться.
 * @tparam Visitor Тип посетителя, который обрабатывает события BFS.
 *
 * @param g Граф, по которому будет выполнен поиск.
 * @param root Корневая вершина для начала поиска.
 * @param visitor Объект посетителя для обработки событий.
 * @param colours Словарь, хранящий цвета вершин.
 */
template <typename Graph, typename Visitor>
void BreadthFirstSearch(
    const Graph& g, typename Graph::vertex_descriptor root, Visitor visitor,
    std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  BreadthFirstSearch(g, std::vector<typename Graph::vertex_descriptor>(1, root),
                     visitor, colours);  // Вызываем BFS с одним корнем
}

#endif  // BREADTH_FIRST_SEARCH_HPP_