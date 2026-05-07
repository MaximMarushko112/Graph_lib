#ifndef BASIC_GRAPH_HPP_
#define BASIC_GRAPH_HPP_

#include <functional>
#include <iterator>

#include "../utils/edge_hash.hpp"

/**
 * @brief Класс, представляющий базовый граф.
 *
 * Данный класс реализует базовую структуру графа, поддерживающую
 * добавление и удаление вершин и рёбер, а также итерацию по ним.
 *
 * @tparam VertexType Тип вершин графа.
 */
template <typename VertexType>
class BasicGraph {
 private:
  template <bool IsConst>
  class VertexIterator;

 public:
  using basic_iterator =
      VertexIterator<false>;  ///< Итератор по вершинам графа.
  using const_basic_iterator =
      VertexIterator<true>;  ///< Константный итератор по вершинам графа.
  using reverse_basic_iterator =
      std::reverse_iterator<basic_iterator>;  ///< Обратный итератор по вершинам
                                              ///< графа.
  using const_reverse_basic_iterator =
      std::reverse_iterator<const_basic_iterator>;  ///< Константный обратный
                                                    ///< итератор по вершинам
                                                    ///< графа.
  using difference_type = std::ptrdiff_t;  ///< Тип для разности итераторов.

  /**
   * @brief Конструктор по умолчанию.
   */
  BasicGraph() = default;

  /**
   * @brief Конструктор копирования.
   *
   * @param g Граф, который будет скопирован.
   */
  BasicGraph(const BasicGraph& g)
      : vertexes_set_(g.vertexes_set_), edges_set_(g.edges_set_) {}

  /**
   * @brief Конструктор перемещения.
   *
   * @param g Граф, который будет перемещен.
   */
  BasicGraph(BasicGraph&& g)
      : vertexes_set_(std::move(g.vertexes_set_)),
        edges_set_(std::move(g.edges_set_)) {}

  /**
   * @brief Оператор присваивания (копирование).
   *
   * @param g Граф, который будет присвоен.
   * @return Ссылка на текущий объект.
   */
  BasicGraph& operator=(const BasicGraph& g) {
    if (&g != this) {
      vertexes_set_ = g.vertexes_set_;
      edges_set_ = g.edges_set_;
    }
    return *this;
  }

  /**
   * @brief Оператор присваивания (перемещение).
   *
   * @param g Граф, который будет перемещен.
   * @return Ссылка на текущий объект.
   */
  BasicGraph& operator=(BasicGraph&& g) {
    if (&g != this) {
      vertexes_set_ = std::move(g.vertexes_set_);
      edges_set_ = std::move(g.edges_set_);
    }
    return *this;
  }

  /**
   * @brief Проверяет, находится ли вершина в графе.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина находится в графе; иначе false.
   */
  bool vertex_in_graph(VertexType* const& vertex) const {
    return vertexes_set_.find(vertex) != vertexes_set_.end();
  }

  /**
   * @brief Проверяет, существует ли ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @return true, если ребро существует; иначе false.
   */
  bool edge_in_graph(VertexType* const& first,
                     VertexType* const& second) const {
    return edges_set_.find({first, second}) != edges_set_.end();
  }

  /**
   * @brief Добавляет вершину в граф.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина была успешно добавлена; иначе false.
   */
  bool add_vertex(VertexType* const& vertex) {
    return vertexes_set_.insert(vertex).second;
  }

  /**
   * @brief Удаляет вершину из графа.
   *
   * @param vertex Указатель на вершину.
   * @return true, если вершина была успешно удалена.
   * @throws std::invalid_argument Если вершина не найдена в графе.
   */
  bool remove_vertex(VertexType* const& vertex) {
    auto vertex_iterator = vertexes_set_.find(vertex);
    if (vertex_iterator == vertexes_set_.end())
      throw std::invalid_argument("There is no such vertex in graph");

    vertexes_set_.erase(vertex_iterator);
    return true;
  }

  /**
   * @brief Добавляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину
   * @param second Указатель на вторую вершину.
   * @param directed Указывает, является ли ребро направленным.
   * @return true, если ребро было успешно добавлено.
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  bool add_edge(VertexType* const& first, VertexType* const& second,
                bool directed) {
    if (!vertex_in_graph(first) || !vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");
    edges_set_.insert({first, second});
    if (!directed) edges_set_.insert({second, first});
    return true;
  }

  /**
   * @brief Удаляет ребро между двумя вершинами.
   *
   * @param first Указатель на первую вершину.
   * @param second Указатель на вторую вершину.
   * @param directed Указывает, является ли ребро направленным.
   * @return true, если ребро было успешно удалено.
   * @throws std::invalid_argument Если одна из вершин не найдена в графе.
   */
  bool remove_edge(VertexType* const& first, VertexType* const& second,
                   bool directed) {
    if (!vertex_in_graph(first) || !vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");
    edges_set_.erase({first, second});
    if (!directed) edges_set_.erase({second, first});
    return true;
  }

  /**
   * @brief Возвращает итератор на начало списка вершин с возможностью
   * фильтрации.
   *
   * @param filter Функция-фильтр для выбора вершин (по умолчанию возвращает все
   * вершины).
   * @return Итератор на начало списка вершин.
   */
  basic_iterator vertexes_begin(
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    return basic_iterator(vertexes_set_.begin(), vertexes_set_.end(), filter);
  }

  /**
   * @brief Возвращает итератор на конец списка вершин с возможностью
   * фильтрации.
   *
   * @param filter Функция-фильтр для выбора вершин (по умолчанию возвращает все
   * вершины).
   * @return Итератор на конец списка вершин.
   */
  basic_iterator vertexes_end(
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    return basic_iterator(vertexes_set_.end(), vertexes_set_.end(), filter);
  }

  /**
   * @brief Возвращает итератор на начало списка рёбер.
   *
   * @return Итератор на начало списка рёбер.
   */
  auto edges_begin() const { return edges_set_.begin(); }

  /**
   * @brief Возвращает итератор на конец списка рёбер.
   *
   * @return Итератор на конец списка рёбер.
   */
  auto edges_end() const { return edges_set_.end(); }

 protected:
  /**
   * @brief Функция-фильтр, которая всегда возвращает true.
   *
   * Используется по умолчанию для итераторов.
   *
   * @param vertex Указатель на вершину.
   * @return true.
   */
  static bool ret_true(VertexType* const&) { return true; }

  std::unordered_set<VertexType*> vertexes_set_;  ///< Множество вершин графа.
  std::unordered_set<std::pair<VertexType*, VertexType*>, EdgeHash<VertexType>>
      edges_set_;  ///< Множество рёбер графа.
};

/**
 * @brief Итератор по вершинам графа.
 *
 * @tparam VertexType Тип вершин графа.
 * @tparam IsConst Указывает, является ли итератор константным.
 */
template <typename VertexType>
template <bool IsConst>
class BasicGraph<VertexType>::VertexIterator {
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
   * @param vertex Итератор на текущую вершину.
   * @param end Итератор на конец множества вершин.
   * @param filter Функция-фильтр для выбора вершин.
   */
  VertexIterator(
      typename std::unordered_set<VertexType*>::const_iterator vertex,
      typename std::unordered_set<VertexType*>::const_iterator end,
      std::function<bool(VertexType* const&)> filter)
      : vertex_(vertex), end_(end), filter_(filter) {
    while (vertex_ != end_ && !filter_(*vertex_)) ++vertex_;
  }

  /**
   * @brief Оператор префиксного инкремента.
   *
   * Перемещает итератор на следующую вершину, удовлетворяющую фильтру.
   *
   * @return Ссылка на текущий итератор.
   */
  VertexIterator& operator++() {
    ++vertex_;
    while (vertex_ != end_ && !filter_(*vertex_)) ++vertex_;
    return *this;
  }

  /**
   * @brief Оператор постфиксного инкремента.
   *
   * Перемещает итератор на следующую вершину, удовлетворяющую фильтру.
   *
   * @return Копия текущего итератора до инкремента.
   */
  VertexIterator operator++(int) {
    VertexIterator copy(*this);
    ++copy;
    return copy;
  }

  /**
   * @brief Оператор разыменования.
   *
   * @return Ссылка на текущую вершину.
   */
  reference operator*() const { return *vertex_; }

  /**
   * @brief Оператор доступа к элементу по указателю.
   *
   * @return Указатель на текущую вершину.
   */
  pointer operator->() const { return *vertex_; }

  /**
   * @brief Оператор сравнения на равенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы равны; иначе false.
   */
  bool operator==(const VertexIterator& other) const {
    return vertex_ == other.vertex_;
  }

  /**
   * @brief Оператор сравнения на неравенство.
   *
   * @param other Другой итератор для сравнения.
   * @return true, если итераторы не равны; иначе false.
   */
  bool operator!=(const VertexIterator& other) const {
    return vertex_ != other.vertex_;
  }

 private:
  typename std::unordered_set<VertexType*>::const_iterator
      vertex_;  ///< Итератор на текущую вершину.
  typename std::unordered_set<VertexType*>::const_iterator
      end_;  ///< Итератор на конец множества вершин.
  std::function<bool(VertexType* const&)>
      filter_;  ///< Функция-фильтр для выбора вершин.
};

#endif  // BASIC_GRAPH_HPP_