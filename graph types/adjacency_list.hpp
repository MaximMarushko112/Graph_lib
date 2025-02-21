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

template <typename VertexType, bool Directed, bool Weighted = false,
          typename WeightType = int>
class AdjasencyList : public BasicGraph<VertexType> {
 private:
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

  AdjasencyList() = default;

  AdjasencyList(const AdjasencyList& g)
      : BasicGraph<VertexType>(g), edges_(g.edges_) {}

  AdjasencyList(AdjasencyList&& g)
      : BasicGraph<VertexType>(g), edges_(std::move(g.edges_)) {}

  AdjasencyList& operator=(const AdjasencyList& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      edges_ = g.edges_;
    }
    return *this;
  }

  AdjasencyList& operator=(AdjasencyList&& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      edges_ = std::move(g.edges_);
    }
    return *this;
  }

  void swap(AdjasencyList& g) {
    AdjasencyList tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  bool add_vertex(VertexType* const& vertex) {
    BasicGraph<VertexType>::add_vertex(vertex);
    return edges_
        .insert({vertex, std::unordered_map<VertexType*, WeightType>()})
        .second;
  }

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

  bool add_edge(VertexType* const& first, VertexType* const& second) {
    if constexpr (Weighted)
      return false;
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

  bool add_edge(VertexType* const& first, VertexType* const& second,
                const WeightType& weight) {
    if constexpr (!Weighted)
      return false;
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

  WeightType edge_weight(VertexType* const& first,
                         VertexType* const& second) const {
    auto first_vertex_adjacency = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);
    if (first_vertex_adjacency == edges_.end() ||
        second_vertex_adjacency == edges_.end())
      throw std::invalid_argument("There are no such vertexes in graph");

    auto edge = first_vertex_adjacency->second.find(second);
    if (edge == first_vertex_adjacency->second.end())
      return std::numeric_limits<WeightType>::max();

    return edge->second;
  }

  iterator neighbours_begin(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    auto vertex_iter = edges_.find(vertex);
    if (vertex_iter == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(vertex_iter->second.begin(), vertex_iter->second.end(),
                    filter);
  }

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
  static bool ret_true(VertexType* const&) { return true; }

  std::unordered_map<VertexType*, std::unordered_map<VertexType*, WeightType>>
      edges_;
};

template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class AdjasencyList<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type =
      std::conditional_t<IsConst, const VertexType*, VertexType*>;
  using reference =
      std::conditional_t<IsConst, const VertexType* const&, VertexType* const&>;
  using pointer = std::conditional_t<IsConst, const VertexType*, VertexType*>;

  Iterator(
      typename std::unordered_map<VertexType*, WeightType>::const_iterator
          neighbour_iter,
      typename std::unordered_map<VertexType*, WeightType>::const_iterator end,
      std::function<bool(VertexType* const&)> filter)
      : neighbour_iter_(neighbour_iter), end_(end), filter_(filter) {
    while (neighbour_iter_ != end_ && !filter_(neighbour_iter_->first))
      ++neighbour_iter_;
  }

  Iterator& operator++() {
    ++neighbour_iter_;
    while (neighbour_iter_ != end_ && !filter_(neighbour_iter_->first))
      ++neighbour_iter_;
    return *this;
  }

  Iterator operator++(int) {
    Iterator copy(*this);
    ++copy;
    return copy;
  }

  reference operator*() const { return neighbour_iter_->first; }

  pointer operator->() const { return neighbour_iter_->first; }

  bool operator==(const Iterator& other) const {
    return neighbour_iter_ == other.neighbour_iter_;
  }

  bool operator!=(const Iterator& other) const {
    return neighbour_iter_ != other.neighbour_iter_;
  }

 private:
  typename std::unordered_map<VertexType*, WeightType>::const_iterator
      neighbour_iter_;
  typename std::unordered_map<VertexType*, WeightType>::const_iterator end_;
  std::function<bool(VertexType* const&)> filter_;
};

#endif  // ADJACENCY_LIST_HPP_