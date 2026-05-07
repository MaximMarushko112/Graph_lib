#ifndef EDGE_LIST_HPP_
#define EDGE_LIST_HPP_

#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "../utils/edge_hash.hpp"
#include "basic_graph.hpp"

/**
 * @brief Класс, представляющий список рёбер графа.
 *
 * Данный класс реализует граф с использованием списка рёбер, поддерживающий
 * как направленные, так и ненаправленные рёбра, а также возможность
 * работы с весами рёбер.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam Directed Указывает, является ли граф направленным.
 * @tparam Weighted Указывает, имеет ли граф веса рёбер.
 * @tparam WeightType Тип весов рёбер (по умолчанию int).
 */
template <typename VertexType, bool Directed, bool Weighted = false,
          typename WeightType = int>
class EdgeList : public BasicGraph<VertexType> {
 private:
  template <bool IsConst>
  class Iterator;

 public:
  using vertex_descriptor = VertexType*;  ///< Указатель на вершину.
  using weight = WeightType;              ///< Тип веса рёбер.
  using edge_hash = EdgeHash<VertexType>;  ///< Хэш-функция для рёбер.
  using iterator = Iterator<false>;  ///< Итератор по рёбрам.
  using const_iterator = Iterator<true>;  ///< Константный итератор по рёбрам.
  using reverse_iterator =
      std::reverse_iterator<iterator>;  ///< Обратный итератор по рёбрам.
  using const_reverse_iterator =
      std::reverse_iterator<const_iterator>;  ///< Константный обратный обратный
                                              ///< итератор по рёбрам.
  using difference_type = std::ptrdiff_t;  ///< Тип для разности итераторов.

  /**
   * @brief Конструктор по умолчанию.
   */
  EdgeList() = default;

  /**
   * @brief Конструктор копирования.
   *
   * @param g Граф, который будет скопирован.
   */
  EdgeList(const EdgeList& g) : BasicGraph<VertexType>(g), edges_(g.edges_) {}

  /**
   * @brief Конструктор перемещения.
   *
   * @param g Граф, который будет перемещен.
   */
  EdgeList(EdgeList&& g)
      : BasicGraph<VertexType>(g), edges_(std::move(g.edges_)) {}

  /**
   * @brief Оператор присваивания (копирование).
   *
   * @param g Граф, который будет присвоен.
   * @return Ссылка на текущий объект.
   */
  EdgeList& operator=(const EdgeList& g) {
    if (&g != this) {
      BasicGraph<VertexType>::operator=(g);
      edges_ = g.edges_;
    }
    return *this;
  }

  /**
   * @brief Оператор присваивания (перемещение).
   *
   * @param g Граф, который будет перемещен.
   * @return Ссылка на текущий объект.
   */
  EdgeList& operator=(EdgeList&& g) {
    if (&g != this) {
      BasicGraph<VertexType>::operator=(g);
      edges_ = std::move(g.edges_);
    }
    return *this;
  }

  /**
   * @brief Обмен содержимым с другим графом.
   *
   * @param g Граф, с которым будет произведен обмен.
   */
  void swap(EdgeList& g) {
    EdgeList tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  /**
   * @brief Добавляет вершину в граф.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина была успешно добавлена; иначе false.
   */
  bool add_vertex(VertexType* const& vertex) {
    return BasicGraph<VertexType>::add_vertex(vertex);
  }

  /**
   * @brief Удаляет вершину из графа.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина была успешно удалена.
   * @throws std::invalid_argument Если вершина не найдена в графе.
   */
  bool remove_vertex(VertexType* const& vertex) {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");

    for (auto neighbour : BasicGraph<VertexType>::vertexes_set_) {
      remove_edge(vertex, neighbour);
      remove_edge(neighbour, vertex);
    }

    return BasicGraph<VertexType>::remove_vertex(vertex);
  }

  /**
   * @brief Добавляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return true, если ребро было успешно добавлено; иначе false.
   */
  bool add_edge(VertexType* const& first, VertexType* const& second) {
    if constexpr (Weighted)
      return false;  // Если граф взвешенный, этот метод не должен
                     // использоваться.
    else {
      BasicGraph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, 1})
          .second;  // Добавляем ребро с весом 1.
    }
    return true;
  }

  /**
   * @brief Добавляет ребро между двумя вершинами с заданным весом.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @param weight Вес ребра.
   * @return true, если ребро было успешно добавлено; иначе false.
   */
  bool add_edge(VertexType* const& first, VertexType* const& second,
                const WeightType& weight) {
    if constexpr (!Weighted)
      return false;  // Если граф не взвешенный, этот метод не должен
                     // использоваться.
    else {
      BasicGraph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, weight})
          .second;  // Добавляем ребро с заданным весом.
    }
    return true;
  }

  /**
   * @brief Удаляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return true, если ребро было успешно удалено; иначе false.
   */
  bool remove_edge(VertexType* const& first, VertexType* const& second) {
    BasicGraph<VertexType>::remove_edge(first, second, Directed);
    auto edge = edges_.find(std::pair{first, second});
    bool edge_found = edge != edges_.end();
    if (edge != edges_.end())
      edges_.erase(edge);
    else if (Directed)
      return false;

    if (!Directed) {
      auto reverse_edge = edges_.find(std::pair{second, first});
      if (!edge_found) edge_found = reverse_edge != edges_.end();
      if (reverse_edge != edges_.end()) edges_.erase(reverse_edge);
    }

    return edge_found;
  }

  /**
   * @brief Возвращает вес ребра между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return Вес ребра между вершинами.
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  WeightType edge_weight(VertexType* const& first,
                         VertexType* const& second) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(first) ||
        !BasicGraph<VertexType>::vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");

    auto edge = edges_.find(std::pair{first, second});
    if (edge != edges_.end()) return edge->second;  // Возвращаем вес ребра.
    if (!Directed) {
      auto reverse_edge = edges_.find(std::pair{second, first});
      if (reverse_edge != edges_.end())
        return reverse_edge->second;  // Возвращаем вес обратного ребра.
    }
    return std::numeric_limits<WeightType>::
        max();  // Если ребро не найдено, возвращаем максимальное значение.
  }

  /**
   * @brief Возвращает итератор на начало списка соседей для заданной вершины.
   *
   * @param vertex Указатель на вершину.
   * @param filter Функция-фильтр для выбора соседей (по умолчанию возвращает
   * всех).
   * @return Итератор на начало списка соседей.
   * @throws std::invalid_argument Если вершина не найдена в графе.
   */
  iterator neighbours_begin(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, BasicGraph<VertexType>::vertexes_begin(),
                    BasicGraph<VertexType>::vertexes_end(), vertex, filter);
  }

  /**
   * @brief Возвращает итератор на конец списка соседей для заданной вершины.
   *
   * @param vertex Указатель на вершину.
   * @param filter Функция-фильтр для выбора соседей (по умолчанию возвращает
   * всех).
   * @return Итератор на конец списка соседей.
   * @throws std::invalid_argument Если вершина не найдена в графе.
   */
  iterator neighbours_end(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, BasicGraph<VertexType>::vertexes_end(),
                    BasicGraph<VertexType>::vertexes_end(), vertex, filter);
  }

 private:
  /**
   * @brief Функция-фильтр, которая всегда возвращает true.
   *
   * Используется по умолчанию для итераторов.
   *
   * @param vertex Указатель на вершину.
   * @return true.
   */
  static bool ret_true(VertexType* const&) { return true; }

  std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType,
                     EdgeHash<VertexType>>
      edges_;  ///< Множество рёбер графа с весами.
};

/**
 * @brief Итератор по соседям для класса EdgeList.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam Directed Указывает, является ли граф направленным.
 * @tparam Weighted Указывает, имеет ли граф веса рёбер.
 * @tparam WeightType Тип весов рёбер.
 */
template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class EdgeList<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category =
      std::forward_iterator_tag;  ///< Категория итератора.
  using value_type = std::conditional_t<IsConst, const VertexType*,
                                        VertexType*>;  ///< Тип значения.
  using reference =
      std::conditional_t<IsConst, const VertexType* const&,
                         VertexType* const&>;  ///< Ссылка на значение.
  using pointer = std::conditional_t<IsConst, const VertexType*,
                                     VertexType*>;  ///< Указатель на значение.

  /**
   * @brief Конструктор итератора.
   *
   * @param edges Указатель на множество рёбер.
   * @param neighbour Итератор на текущего соседа.
   * @param end Итератор на конец множества соседей.
   * @param vertex Указатель на вершину, для которой ищутся соседи.
   * @param filter Функция-фильтр для выбора соседей.
   */
  Iterator(const std::unordered_map<std::pair<VertexType*, VertexType*>,
                                    WeightType, EdgeHash<VertexType>>* edges,
           typename BasicGraph<VertexType>::basic_iterator neighbour,
           typename BasicGraph<VertexType>::basic_iterator end,
           VertexType* vertex, std::function<bool(VertexType* const&)> filter)
      : edges_(edges),
        neighbour_(neighbour),
        end_(end),
        vertex_(vertex),
        filter_(filter) {
    while (neighbour_ != end_ &&
           (!filter_(*neighbour_) ||
            (edges_->find({vertex_, *neighbour_}) == edges_->end() &&
             edges_->find({*neighbour_, vertex_}) == edges_->end())))
      ++neighbour_;
  }

  /**
   * @brief Оператор префиксного инкремента.
   *
   * Перемещает итератор на следующего соседа, удовлетворяющего фильтру.
   *
   * @return Ссылка на текущий итератор.
   */
  Iterator& operator++() {
    ++neighbour_;
    while (neighbour_ != end_ &&
           ((edges_->find({vertex_, *neighbour_}) == edges_->end() &&
             edges_->find({*neighbour_, vertex_}) == edges_->end()) ||
            !filter_(*neighbour_)))
      ++neighbour_;
    return *this;
  }

  /**
   * @brief Оператор постфиксного инкремента.
   *
   * @return Копия текущего итератора до инкремента.
   */
  Iterator operator++(int) {
    Iterator copy(*this);
    ++copy;
    return copy;
  }

  /**
   * @brief Оператор разыменования.
   *
   * @return Ссылка на текущего соседа.
   */
  reference operator*() const { return *neighbour_; }

  /**
   * @brief Оператор доступа к элементу по указателю.
   *
   * @return Указатель на текущего соседа.
   */
  pointer operator->() const { return *neighbour_; }

  /**
   * @brief Оператор сравнения на равенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы равны; иначе false.
   */
  bool operator==(const Iterator& other) const {
    return neighbour_ == other.neighbour_ && vertex_ == other.vertex_;
  }

  /**
   * @brief Оператор сравнения на неравенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы не равны; иначе false.
   */
  bool operator!=(const Iterator& other) const {
    return neighbour_ != other.neighbour_;
  }

 private:
  const std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType,
                           EdgeHash<VertexType>>*
      edges_;  ///< Указатель на множество рёбер.
  typename BasicGraph<VertexType>::basic_iterator
      neighbour_;  ///< Итератор на текущего соседа.
  typename BasicGraph<VertexType>::basic_iterator
      end_;  ///< Итератор на конец множества соседей.
  VertexType* vertex_;  ///< Указатель на вершину, для которой ищутся соседи.
  std::function<bool(VertexType* const&)>
      filter_;  ///< Функция-фильтр для выбора соседей.
};

#endif  // EDGE_LIST_HPP_