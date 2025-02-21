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

template <typename VertexType, bool Directed, bool Weighted = false,
          typename WeightType = int>
class AdjacencyMatrix : public BasicGraph<VertexType> {
 private:
  struct Edge {
    bool is_exist;
    WeightType weight;
  };

  template <bool IsConst>
  class Iterator;

 public:
  using vertex_descriptor = VertexType*;
  using weight = WeightType;
  using edge_hash = EdgeHash<VertexType>;
  using iterator = Iterator<false>;
  using const_iterator = Iterator<true>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
  using difference_type = std::ptrdiff_t;

  AdjacencyMatrix(std::size_t n)
      : vertexes_(n),
        matrix_(n, std::vector<Edge>(n)),
        vertex_count_(),
        max_vertex_count_(n) {}

  AdjacencyMatrix(const AdjacencyMatrix& g)
      : BasicGraph<VertexType>(g),
        indexes_(g.indexes_),
        vertexes_(g.vertexes_),
        matrix_(g.matrix_),
        free_indexes_(g.free_indexes_),
        vertex_count_(g.vertex_count_),
        max_vertex_count_(g.max_vertex_count_) {}

  AdjacencyMatrix(AdjacencyMatrix&& g)
      : BasicGraph<VertexType>(g),
        indexes_(std::move(g.indexes_)),
        vertexes_(std::move(g.vertexes_)),
        matrix_(std::move(g.matrix_)),
        free_indexes_(std::move(g.free_indexes_)),
        vertex_count_(std::move(g.vertex_count_)),
        max_vertex_count_(std::move(g.max_vertex_count_)) {}

  AdjacencyMatrix& operator=(const AdjacencyMatrix& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      indexes_ = g.indexes_;
      vertexes_ = g.vertexes_;
      matrix_ = g.matrix_;
      free_indexes_ = g.free_indexes_;
      vertex_count_ = g.vertex_count_;
      max_vertex_count_ = g.max_vertex_count_;
    }
    return *this;
  }

  AdjacencyMatrix& operator=(AdjacencyMatrix&& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      indexes_ = std::move(g.indexes_);
      vertexes_ = std::move(g.vertexes_);
      matrix_ = std::move(g.matrix_);
      free_indexes_ = std::move(g.free_indexes_);
      vertex_count_ = std::move(g.vertex_count_);
      max_vertex_count_ = std::move(g.max_vertex_count_);
    }
    return *this;
  }

  void swap(AdjacencyMatrix& g) {
    AdjacencyMatrix tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  bool add_vertex(VertexType* const& vertex) {
    std::size_t index = free_index();
    if (!indexes_.insert({vertex, index}).second) return false;
    BasicGraph<VertexType>::add_vertex(vertex);
    vertexes_[index] = vertex;
    ++vertex_count_;
    return true;
  }

  bool remove_vertex(VertexType* const& vertex) {
    auto vertex_iterator = indexes_.find(vertex);
    if (vertex_iterator == indexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    std::size_t vertex_index = vertex_iterator->second;

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

  bool add_edge(VertexType* const& first, VertexType* const& second) {
    if constexpr (Weighted)
      return false;
    else {
      auto first_vertex_index = indexes_.find(first);
      auto second_vertex_index = indexes_.find(second);
      if (first_vertex_index == indexes_.end() ||
          second_vertex_index == indexes_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      matrix_[first_vertex_index->second][second_vertex_index->second] =
          Edge{true, 1};
      if (!Directed) {
        matrix_[second_vertex_index->second][first_vertex_index->second] =
            Edge{true, 1};
      }
      BasicGraph<VertexType>::add_edge(first, second, Directed);
    }
    return true;
  }

  bool add_edge(VertexType* const& first, VertexType* const& second,
                const WeightType& weight) {
    if constexpr (!Weighted)
      return false;
    else {
      auto first_vertex_index = indexes_.find(first);
      auto second_vertex_index = indexes_.find(second);
      if (first_vertex_index == indexes_.end() ||
          second_vertex_index == indexes_.end())
        throw std::invalid_argument("There are no such vertexes in graph");

      matrix_[first_vertex_index->second][second_vertex_index->second] =
          Edge{true, weight};
      if (!Directed) {
        matrix_[second_vertex_index->second][first_vertex_index->second] =
            Edge{true, weight};
      }
      BasicGraph<VertexType>::add_edge(first, second, Directed);
    }
    return true;
  }

  bool remove_edge(VertexType* const& first, VertexType* const& second) {
    auto first_vertex_index = indexes_.find(first);
    auto second_vertex_index = indexes_.find(second);
    if (first_vertex_index == indexes_.end() ||
        second_vertex_index == indexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    bool edge_found =
        matrix_[first_vertex_index->second][second_vertex_index->second]
            .is_exist;
    matrix_[first_vertex_index->second][second_vertex_index->second] =
        Edge{false, WeightType()};
    if (!Directed) {
      matrix_[second_vertex_index->second][first_vertex_index->second] =
          Edge{false, WeightType()};
    }
    BasicGraph<VertexType>::remove_edge(first, second, Directed);

    return edge_found;
  }

  WeightType edge_weight(VertexType* const& first,
                         VertexType* const& second) const {
    auto first_vertex_index = indexes_.find(first);
    auto second_vertex_index = indexes_.find(second);
    if (first_vertex_index == indexes_.end() ||
        second_vertex_index == indexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    if (!matrix_[first_vertex_index->second][second_vertex_index->second]
             .is_exist)
      return std::numeric_limits<WeightType>::max();
    return matrix_[first_vertex_index->second][second_vertex_index->second]
        .weight;
  }

  iterator neighbours_begin(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    auto vertex_iter = indexes_.find(vertex);
    if (vertex_iter == indexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&vertexes_, &matrix_[vertex_iter->second], 0, filter);
  }

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
  static bool ret_true(VertexType* const&) { return true; }

  std::size_t free_index() {
    if (free_indexes_.empty()) {
      if (vertex_count_ >= max_vertex_count_)
        throw std::length_error("Too much vertexes_");
      return vertex_count_;
    }
    std::size_t index = free_indexes_.top();
    free_indexes_.pop();
    return index;
  }

 private:
  std::vector<VertexType*> vertexes_;
  std::unordered_map<VertexType*, std::size_t> indexes_;
  std::vector<std::vector<Edge>> matrix_;
  std::stack<std::size_t> free_indexes_;
  std::size_t vertex_count_;
  std::size_t max_vertex_count_;
};

template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class AdjacencyMatrix<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type =
      std::conditional_t<IsConst, const VertexType*, VertexType*>;
  using reference =
      std::conditional_t<IsConst, const VertexType*&, VertexType* const&>;
  using pointer = std::conditional_t<IsConst, const VertexType*, VertexType*>;

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

  Iterator& operator++() {
    ++neighbour_index_;
    while (neighbour_index_ != adjacency_->size() &&
           (!(*adjacency_)[neighbour_index_].is_exist ||
            !filter_((*vertexes_)[neighbour_index_])))
      ++neighbour_index_;
    return *this;
  }

  Iterator operator++(int) {
    Iterator copy(*this);
    ++copy;
    return copy;
  }

  reference operator*() const { return (*vertexes_)[neighbour_index_]; }

  pointer operator->() const { return (*vertexes_)[neighbour_index_]; }

  bool operator==(const Iterator& other) const {
    return neighbour_index_ == other.neighbour_iter_ &&
           adjacency_ == other.adjacency_;
  }

  bool operator!=(const Iterator& other) const {
    return neighbour_index_ != other.neighbour_index_ ||
           adjacency_ != other.adjacency_;
  }

 private:
  const std::vector<VertexType*>* vertexes_;
  const std::vector<Edge>* adjacency_;
  std::size_t neighbour_index_;
  std::function<bool(VertexType* const&)> filter_;
};

#endif  // ADJACENCY_MATRIX_HPP_