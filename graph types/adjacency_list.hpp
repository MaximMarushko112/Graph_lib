#ifndef ADJACENCY_LIST_HPP_
#define ADJACENCY_LIST_HPP_

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
 * @brief Класс, представляющий список смежности графа.
 *
 * Данный класс реализует граф с использованием списка смежности, поддерживающий
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
class AdjacencyList : public BasicGraph<VertexType> {
 private:
  template <bool IsConst>
  class Iterator;

 public:
  using vertex_descriptor = VertexType*;  ///< Указатель на вершину.
  using weight = WeightType;              ///< Тип веса рёбер.
  using edge_hash = EdgeHash<VertexType>;  ///< Хэш-функция для рёбер.
  using iterator = Iterator<false>;  ///< Итератор по соседям.
  using const_iterator = Iterator<true>;  ///< Константный итератор по соседям.
  using reverse_iterator =
      std::reverse_iterator<iterator>;  ///< Обратный итератор по соседям.
  using const_reverse_iterator =
      std::reverse_iterator<const_iterator>;  ///< Константный обратный обратный
                                              ///< итератор по соседям.
  using difference_type = std::ptrdiff_t;  ///< Тип для разности итераторов.

  /**
   * @brief Конструктор по умолчанию.
   */
  AdjacencyList() = default;

  /**
   * @brief Конструктор копирования.
   *
   * @param g Граф, который будет скопирован.
   */
  AdjacencyList(const AdjacencyList& g)
      : BasicGraph<VertexType>(g), edges_(g.edges_) {}

  /**
   * @brief Конструктор перемещения.
   *
   * @param g Граф, который будет перемещен.
   */
  AdjacencyList(AdjacencyList&& g)
      : BasicGraph<VertexType>(g), edges_(std::move(g.edges_)) {}

  /**
   * @brief Оператор присваивания (копирование).
   *
   * @param g Граф, который будет присвоен.
   * @return Ссылка на текущий объект.
   */
  AdjacencyList& operator=(const AdjacencyList& g) {
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
  AdjacencyList& operator=(AdjacencyList&& g) {
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
  void swap(AdjacencyList& g) {
    AdjacencyList tmp(std::move(g));
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
    BasicGraph<VertexType>::add_vertex(vertex);
    return edges_
        .insert({vertex, std::unordered_map<VertexType*, WeightType>()})
        .second;
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

    for (auto other_vertex : edges_) {
      remove_edge(vertex, other_vertex.first);
      if (Directed) remove_edge(other_vertex.first, vertex);
    }

    BasicGraph<VertexType>::remove_vertex(vertex);
    edges_.erase(edges_.find(vertex));
    return true;
  }

  /**
   * @brief Добавляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return true, если ребро было успешно добавлено; иначе false.
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  bool add_edge(VertexType* const& first, VertexType* const& second) {
    if constexpr (Weighted)
      return false;  // Если граф взвешенный, этот метод не должен
                     // использоваться.
    else {
      auto first_vertex_adjacency = edges_.find(first);
      auto second_vertex_adjacency = edges_.find(second);
      if (first_vertex_adjacency == edges_.end() ||
          second_vertex_adjacency == edges_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      first_vertex_adjacency->second.insert({second, 1});
      if (!Directed) {
        second_vertex_adjacency->second.insert({first, 1});
      }
      BasicGraph<VertexType>::add_edge(first, second, Directed);
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
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  bool add_edge(VertexType* const& first, VertexType* const& second,
                const WeightType& weight) {
    if constexpr (!Weighted)
      return false;  // Если граф не взвешенный, этот метод не должен
                     // использоваться.
    else {
      auto first_vertex_adjacency = edges_.find(first);
      auto second_vertex_adjacency = edges_.find(second);
      if (first_vertex_adjacency == edges_.end() ||
          second_vertex_adjacency == edges_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      first_vertex_adjacency->second.insert({second, weight});
      if (!Directed) {
        second_vertex_adjacency->second.insert({first, weight});
      }
      BasicGraph<VertexType>::add_edge(first, second, Directed);
    }
    return true;
  }

  /**
   * @brief Удаляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return true, если ребро было успешно удалено; иначе false.
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  bool remove_edge(VertexType* const& first, VertexType* const& second) {
    auto first_vertex_adjacency = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);
    if (first_vertex_adjacency == edges_.end() ||
        second_vertex_adjacency == edges_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    auto edge = first_vertex_adjacency->second.find(second);
    if (edge == first_vertex_adjacency->second.end()) return false;
    first_vertex_adjacency->second.erase(edge);

    if (!Directed) {
      auto reverse_edge = second_vertex_adjacency->second.find(first);
      if (reverse_edge == second_vertex_adjacency->second.end()) return false;
      second_vertex_adjacency->second.erase(reverse_edge);
    }

    BasicGraph<VertexType>::remove_edge(first, second, Directed);

    return true;
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
    auto first_vertex_adjacency = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);
    if (first_vertex_adjacency == edges_.end() ||
        second_vertex_adjacency == edges_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    auto edge = first_vertex_adjacency->second.find(second);
    if (edge == first_vertex_adjacency->second.end())
      return std::numeric_limits<WeightType>::
          max();  // Если ребро не найдено, возвращаем максимальное значение.

    return edge->second;  // Возвращаем вес ребра.
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
    auto vertex_iter = edges_.find(vertex);
    if (vertex_iter == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(vertex_iter->second.begin(), vertex_iter->second.end(),
                    filter);
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
    auto vertex_iter = edges_.find(vertex);
    if (vertex_iter == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(vertex_iter->second.end(), vertex_iter->second.end(),
                    filter);
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

  std::unordered_map<VertexType*, std::unordered_map<VertexType*, WeightType>>
      edges_;  ///< Список смежности, хранящий рёбра и их веса.
};

/**
 * @brief Итератор по соседям для класса AdjacencyList.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam Directed Указывает, является ли граф направленным.
 * @tparam Weighted Указывает, имеет ли граф веса рёбер.
 * @tparam WeightType Тип весов рёбер.
 */
template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class AdjacencyList<VertexType, Directed, Weighted, WeightType>::Iterator {
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
   * @param neighbour_iter Итератор на текущего соседа.
   * @param end Итератор на конец списка соседей.
   * @param filter Функция-фильтр для выбора соседей.
   */
  Iterator(
      typename std::unordered_map<VertexType*, WeightType>::const_iterator
          neighbour_iter,
      typename std::unordered_map<VertexType*, WeightType>::const_iterator end,
      std::function<bool(VertexType* const&)> filter)
      : neighbour_iter_(neighbour_iter), end_(end), filter_(filter) {
    while (neighbour_iter_ != end_ && !filter_(neighbour_iter_->first))
      ++neighbour_iter_;
  }

  /**
   * @brief Оператор префиксного инкремента.
   *
   * Перемещает итератор на следующего соседа, удовлетворяющего фильтру.
   *
   * @return Ссылка на текущий итератор.
   */
  Iterator& operator++() {
    ++neighbour_iter_;
    while (neighbour_iter_ != end_ && !filter_(neighbour_iter_->first))
      ++neighbour_iter_;
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
  reference operator*() const { return neighbour_iter_->first; }

  /**
   * @brief Оператор доступа к элементу по указателю.
   *
   * @return Указатель на текущего соседа.
   */
  pointer operator->() const { return neighbour_iter_->first; }

  /**
   * @brief Оператор сравнения на равенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы равны; иначе false.
   */
  bool operator==(const Iterator& other) const {
    return neighbour_iter_ == other.neighbour_iter_;
  }

  /**
   * @brief Оператор сравнения на неравенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы не равны; иначе false.
   */
  bool operator!=(const Iterator& other) const {
    return neighbour_iter_ != other.neighbour_iter_;
  }

 private:
  typename std::unordered_map<VertexType*, WeightType>::const_iterator
      neighbour_iter_;  ///< Итератор на текущего соседа.
  typename std::unordered_map<VertexType*, WeightType>::const_iterator
      end_;  ///< Итератор на конец списка соседей.
  std::function<bool(VertexType* const&)>
      filter_;  ///< Функция-фильтр для выбора соседей.
};

#endif  // ADJACENCY_LIST_HPP_