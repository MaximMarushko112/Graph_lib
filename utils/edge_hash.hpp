#ifndef EDGE_HASH_HPP_
#define EDGE_HASH_HPP_

#include <utility>

/**
 * @brief Структура для вычисления хэш-значения для ребер графа.
 *
 * Данная структура реализует хэш-функцию для пары указателей на вершины графа.
 * Хэш-функция используется для уникальной идентификации ребер в графе.
 *
 * @tparam VertexType Тип вершин графа.
 */
template <typename VertexType>
struct EdgeHash {
  /**
   * @brief Вычисляет хэш-значение для пары указателей на вершины.
   *
   * Данная функция принимает пару указателей на вершины и возвращает
   * хэш-значение, вычисленное на основе указателей. Хэш-значение
   * вычисляется с использованием стандартной хэш-функции для указателей
   * и комбинируется с помощью операции XOR.
   *
   * @param edge Пара указателей на вершины графа.
   * @return std::size_t Хэш-значение для данной пары указателей.
   */
  std::size_t operator()(
      const std::pair<VertexType*, VertexType*>& edge) const noexcept {
    std::size_t h1 = std::hash<VertexType*>{}(edge.first);
    std::size_t h2 = std::hash<VertexType*>{}(edge.second);
    return h1 ^ (h2 << 1);
  }
};

#endif  // EDGE_HASH_HPP_