#ifndef ADJACENCY_MATRIX_HPP_
#define ADJACENCY_MATRIX_HPP_

#include <functional>
#include <iterator>
#include <limits>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../utils/edge_hash.hpp"
#include "basic_graph.hpp"

/**
 * @brief Класс, представляющий матрицу смежности графа.
 *
 * Данный класс реализует граф с использованием матрицы смежности,
 * поддерживающий как направленные, так и ненаправленные рёбра, а также
 * возможность работы с весами рёбер.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam Directed Указывает, является ли граф направленным.
 * @tparam Weighted Указывает, имеет ли граф веса рёбер.
 * @tparam WeightType Тип весов рёбер (по умолчанию int).
 */
template <typename VertexType, bool Directed, bool Weighted = false,
          typename WeightType = int>
class AdjacencyMatrix : public BasicGraph<VertexType> {
 private:
  struct Edge {
    bool is_exist;  ///< Указывает, существует ли ребро.
    WeightType weight;  ///< Вес ребра.
  };

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
   * @brief Конструктор, создающий матрицу смежности заданного размера.
   *
   * @param n Размер матрицы (количество вершин).
   */
  AdjacencyMatrix(std::size_t n)
      : vertexes_(n),
        matrix_(n, std::vector<Edge>(n)),
        vertex_count_(),
        max_vertex_count_(n) {}

  /**
   * @brief Конструктор копирования.
   *
   * @param g Граф, который будет скопирован.
   */
  AdjacencyMatrix(const AdjacencyMatrix& g)
      : BasicGraph<VertexType>(g),
        indexes_(g.indexes_),
        vertexes_(g.vertexes_),
        matrix_(g.matrix_),
        free_indexes_(g.free_indexes_),
        vertex_count_(g.vertex_count_),
        max_vertex_count_(g.max_vertex_count_) {}

  /**
   * @brief Конструктор перемещения.
   *
   * @param g Граф, который будет перемещен.
   */
  AdjacencyMatrix(AdjacencyMatrix&& g)
      : BasicGraph<VertexType>(g),
        indexes_(std::move(g.indexes_)),
        vertexes_(std::move(g.vertexes_)),
        matrix_(std::move(g.matrix_)),
        free_indexes_(std::move(g.free_indexes_)),
        vertex_count_(std::move(g.vertex_count_)),
        max_vertex_count_(std::move(g.max_vertex_count_)) {}

  /**
   * @brief Оператор присваивания (копирование).
   *
   * @param g Граф, который будет присвоен.
   * @return Ссылка на текущий объект.
   */
  AdjacencyMatrix& operator=(const AdjacencyMatrix& g) {
    if (&g != this) {
      BasicGraph<VertexType>::operator=(g);
      indexes_ = g.indexes_;
      vertexes_ = g.vertexes_;
      matrix_ = g.matrix_;
      free_indexes_ = g.free_indexes_;
      vertex_count_ = g.vertex_count_;
      max_vertex_count_ = g.max_vertex_count_;
    }
    return *this;
  }

  /**
   * @brief Оператор присваивания (перемещение).
   *
   * @param g Граф, который будет перемещен.
   * @return Ссылка на текущий объект.
   */
  AdjacencyMatrix& operator=(AdjacencyMatrix&& g) {
    if (&g != this) {
      BasicGraph<VertexType>::operator=(g);
      indexes_ = std::move(g.indexes_);
      vertexes_ = std::move(g.vertexes_);
      matrix_ = std::move(g.matrix_);
      free_indexes_ = std::move(g.free_indexes_);
      vertex_count_ = std::move(g.vertex_count_);
      max_vertex_count_ = std::move(g.max_vertex_count_);
    }
    return *this;
  }

  /**
   * @brief Обмен содержимым с другим графом.
   *
   * @param g Граф, с которым будет произведен обмен.
   */
  void swap(AdjacencyMatrix& g) {
    AdjacencyMatrix tmp(std::move(g));
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
    std::size_t index = free_index();
    if (!indexes_.insert({vertex, index}).second) return false;
    BasicGraph<VertexType>::add_vertex(vertex);
    vertexes_[index] = vertex;
    ++vertex_count_;
    return true;
  }

  /**
   * @brief Удаляет вершину из графа.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина была успешно удалена.
   * @throws std::invalid_argument Если вершина не найдена в графе.
   */
  bool remove_vertex(VertexType* const& vertex) {
    auto vertex_iterator = indexes_.find(vertex);
    if (vertex_iterator == indexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    std::size_t vertex_index = vertex_iterator->second;

    // Удаление всех рёбер, связанных с удаляемой вершиной
    for (std::size_t other_index = 0; other_index < max_vertex_count_;
         ++other_index) {
      matrix_[vertex_index][other_index] = Edge{false, WeightType()};
      matrix_[other_index][vertex_index] = Edge{false, WeightType()};
    }

    for (auto neighbour : BasicGraph<VertexType>::vertexes_set_) {
      BasicGraph<VertexType>::remove_edge(vertex, neighbour, Directed);
      BasicGraph<VertexType>::remove_edge(neighbour, vertex, Directed);
    }

    BasicGraph<VertexType>::remove_vertex(vertex);
    vertexes_[vertex_index] = nullptr;
    indexes_.erase(vertex_iterator);
    free_indexes_.push(vertex_index);
    --vertex_count_;
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
      auto first_vertex_index = indexes_.find(first);
      auto second_vertex_index = indexes_.find(second);
      if (first_vertex_index == indexes_.end() ||
          second_vertex_index == indexes_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      matrix_[first_vertex_index->second][second_vertex_index->second] =
          Edge{true, 1};  // Добавляем ребро с весом 1.
      if (!Directed) {
        matrix_[second_vertex_index->second][first_vertex_index->second] = Edge{
            true, 1};  // Добавляем обратное ребро, если граф ненаправленный.
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
      auto first_vertex_index = indexes_.find(first);
      auto second_vertex_index = indexes_.find(second);
      if (first_vertex_index == indexes_.end() ||
          second_vertex_index == indexes_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      matrix_[first_vertex_index->second][second_vertex_index->second] =
          Edge{true, weight};  // Добавляем ребро с заданным весом.
      if (!Directed) {
        matrix_[second_vertex_index->second][first_vertex_index->second] = Edge{
            true,
            weight};  // Добавляем обратное ребро, если граф ненаправленный.
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
    auto first_vertex_index = indexes_.find(first);
    auto second_vertex_index = indexes_.find(second);
    if (first_vertex_index == indexes_.end() ||
        second_vertex_index == indexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    bool edge_found =
        matrix_[first_vertex_index->second][second_vertex_index->second]
            .is_exist;  // Проверяем, существует ли ребро.
    matrix_[first_vertex_index->second][second_vertex_index->second] =
        Edge{false, WeightType()};  // Удаляем ребро.
    if (!Directed) {
      matrix_[second_vertex_index->second][first_vertex_index->second] = Edge{
          false,
          WeightType()};  // Удаляем обратное ребро, если граф ненаправленный.
    }
    BasicGraph<VertexType>::remove_edge(first, second, Directed);

    return edge_found;  // Возвращаем, было ли найдено ребро.
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
    auto first_vertex_index = indexes_.find(first);
    auto second_vertex_index = indexes_.find(second);
    if (first_vertex_index == indexes_.end() ||
        second_vertex_index == indexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    if (!matrix_[first_vertex_index->second][second_vertex_index->second]
             .is_exist)
      return std::numeric_limits<WeightType>::
          max();  // Если ребро не найдено, возвращаем максимальное значение.
    return matrix_[first_vertex_index->second][second_vertex_index->second]
        .weight;  // Возвращаем вес ребра.
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
    auto vertex_iter = indexes_.find(vertex);
    if (vertex_iter == indexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&vertexes_, &matrix_[vertex_iter->second], 0, filter);
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
    auto vertex_iter = indexes_.find(vertex);
    if (vertex_iter == indexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&vertexes_, &matrix_[vertex_iter->second],
                    max_vertex_count_, filter);
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

  /**
   * @brief Возвращает свободный индекс для новой вершины.
   *
   * @return Свободный индекс.
   * @throws std::length_error Если превышено максимальное количество вершин.
   */
  std::size_t free_index() {
    if (free_indexes_.empty()) {
      if (vertex_count_ >= max_vertex_count_)
        throw std::length_error("Too much vertexes");
      return vertex_count_;
    }
    std::size_t index = free_indexes_.top();
    free_indexes_.pop();
    return index;
  }

 private:
  std::vector<VertexType*> vertexes_;  ///< Вектор вершин графа.
  std::unordered_map<VertexType*, std::size_t> indexes_;  ///< Индексы вершин.
  std::vector<std::vector<Edge>> matrix_;  ///< Матрица смежности.
  std::stack<std::size_t> free_indexes_;  ///< Стек свободных индексов.
  std::size_t vertex_count_;  ///< Текущее количество вершин.
  std::size_t max_vertex_count_;  ///< Максимальное количество вершин.
};

/**
 * @brief Итератор по соседям для класса AdjacencyMatrix.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam Directed Указывает, является ли граф направленным.
 * @tparam Weighted Указывает, имеет ли граф веса рёбер.
 * @tparam WeightType Тип весов рёбер.
 */
template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class AdjacencyMatrix<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category =
      std::forward_iterator_tag;  ///< Категория итератора.
  using value_type = std::conditional_t<IsConst, const VertexType*,
                                        VertexType*>;  ///< Тип значения.
  using reference =
      std::conditional_t<IsConst, const VertexType*&,
                         VertexType* const&>;  ///< Ссылка на значение.
  using pointer = std::conditional_t<IsConst, const VertexType*,
                                     VertexType*>;  ///< Указатель на значение.

  /**
   * @brief Конструктор итератора.
   *
   * @param vertexes Указатель на вектор вершин.
   * @param adjacency Указатель на вектор рёбер (матрицу смежности).
   * @param neighbour_index Индекс текущего соседа.
   * @param filter Функция-фильтр для выбора соседей.
   */
  Iterator(const std::vector<VertexType*>* vertexes,
           const std::vector<Edge>* adjacency, std::size_t neighbour_index,
           std::function<bool(VertexType* const&)> filter)
      : vertexes_(vertexes),
        adjacency_(adjacency),
        neighbour_index_(neighbour_index),
        filter_(filter) {
    while (neighbour_index_ != adjacency_->size() &&
           (!(*adjacency_)[neighbour_index_].is_exist ||
            !filter_((*vertexes_)[neighbour_index_])))
      ++neighbour_index_;
  }

  /**
   * @brief Оператор префиксного инкремента.
   *
   * Перемещает итератор на следующего соседа, удовлетворяющего фильтру.
   *
   * @return Ссылка на текущий итератор.
   */
  Iterator& operator++() {
    ++neighbour_index_;
    while (neighbour_index_ != adjacency_->size() &&
           (!(*adjacency_)[neighbour_index_].is_exist ||
            !filter_((*vertexes_)[neighbour_index_])))
      ++neighbour_index_;
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
  reference operator*() const { return (*vertexes_)[neighbour_index_]; }

  /**
   * @brief Оператор доступа к элементу по указателю.
   *
   * @return Указатель на текущего соседа.
   */
  pointer operator->() const { return (*vertexes_)[neighbour_index_]; }

  /**
   * @brief Оператор сравнения на равенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы равны; иначе false.
   */
  bool operator==(const Iterator& other) const {
    return neighbour_index_ == other.neighbour_index_ &&
           adjacency_ == other.adjacency_;
  }

  /**
   * @brief Оператор сравнения на неравенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы не равны; иначе false.
   */
  bool operator!=(const Iterator& other) const {
    return neighbour_index_ != other.neighbour_index_ ||
           adjacency_ != other.adjacency_;
  }

 private:
  const std::vector<VertexType*>* vertexes_;  ///< Указатель на вектор вершин.
  const std::vector<Edge>*
      adjacency_;  ///< Указатель на вектор рёбер (матрицу смежности).
  std::size_t neighbour_index_;  ///< Индекс текущего соседа.
  std::function<bool(VertexType* const&)>
      filter_;  ///< Функция-фильтр для выбора соседей.
};

#endif  // ADJACENCY_MATRIX_HPP_