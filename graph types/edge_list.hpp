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

template <typename VertexType, bool Directed, bool Weighted = false,
          typename WeightType = int>
class EdgeList : public BasicGraph<VertexType> {
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

  EdgeList() = default;

  EdgeList(const EdgeList& g) : BasicGraph<VertexType>(g), edges_(g.edges_) {}

  EdgeList(EdgeList&& g)
      : BasicGraph<VertexType>(g), edges_(std::move(g.edges_)) {}

  EdgeList& operator=(const EdgeList& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      edges_ = g.edges_;
    }
    return *this;
  }

  EdgeList& operator=(EdgeList&& g) {
    if (&g != this) {
      BasicGraph<VertexType>(g);
      edges_ = std::move(g.edges_);
    }
    return *this;
  }

  void swap(EdgeList& g) {
    EdgeList tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  bool add_vertex(VertexType* const& vertex) {
    return BasicGraph<VertexType>::add_vertex(vertex);
  }

  bool remove_vertex(VertexType* const& vertex) {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");

    for (auto neighbour : BasicGraph<VertexType>::vertexes_set_) {
      remove_edge(vertex, neighbour);
      remove_edge(neighbour, vertex);
    }

    return BasicGraph<VertexType>::remove_vertex(vertex);
  }

  bool add_edge(VertexType* const& first, VertexType* const& second) {
    if constexpr (Weighted)
      return false;
    else {
      BasicGraph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, 1}).second;
    }
    return true;
  }

  bool add_edge(VertexType* const& first, VertexType* const& second,
                const WeightType& weight) {
    if constexpr (!Weighted)
      return false;
    else {
      BasicGraph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, weight}).second;
    }
    return true;
  }

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
      if (reverse_edge != edges_.end()) edges_.erase(edge);
    }

    return edge_found;
  }

  WeightType edge_weight(VertexType* const& first,
                         VertexType* const& second) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(first) ||
        !BasicGraph<VertexType>::vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");

    auto edge = edges_.find(std::pair{first, second});
    if (edge != edges_.end())
      return edges_.find(std::pair{first, second})->second;
    if (!Directed) {
      auto reverse_edge = edges_.find(std::pair{second, first});
      if (reverse_edge != edges_.end())
        return edges_.find(std::pair{second, first})->second;
    }
    return std::numeric_limits<WeightType>::max();
  }

  iterator neighbours_begin(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, BasicGraph<VertexType>::vertexes_begin(),
                    BasicGraph<VertexType>::vertexes_end(), vertex, filter);
  }

  iterator neighbours_end(
      VertexType* const& vertex,
      std::function<bool(VertexType* const&)> filter = ret_true) const {
    if (!BasicGraph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, BasicGraph<VertexType>::vertexes_end(),
                    BasicGraph<VertexType>::vertexes_end(), vertex, filter);
  }

 private:
  static bool ret_true(VertexType* const&) { return true; }

  std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType,
                     EdgeHash<VertexType>>
      edges_;
};

template <typename VertexType, bool Directed, bool Weighted,
          typename WeightType>
template <bool IsConst>
class EdgeList<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type =
      std::conditional_t<IsConst, const VertexType*, VertexType*>;
  using reference =
      std::conditional_t<IsConst, const VertexType* const&, VertexType* const&>;
  using pointer = std::conditional_t<IsConst, const VertexType*, VertexType*>;

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

  Iterator& operator++() {
    ++neighbour_;
    while (neighbour_ != end_ &&
           ((edges_->find({vertex_, *neighbour_}) == edges_->end() &&
             edges_->find({*neighbour_, vertex_}) == edges_->end()) ||
            !filter_(*neighbour_)))
      ++neighbour_;
    return *this;
  }

  Iterator operator++(int) {
    Iterator copy(*this);
    ++copy;
    return copy;
  }

  reference operator*() const { return *neighbour_; }

  pointer operator->() const { return *neighbour_; }

  bool operator==(const Iterator& other) const {
    return neighbour_ == other.neighbour_ && vertex_ == other.vertex_;
  }

  bool operator!=(const Iterator& other) const {
    return neighbour_ != other.neighbour_;
  }

 private:
  const std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType,
                           EdgeHash<VertexType>>* edges_;
  typename BasicGraph<VertexType>::basic_iterator neighbour_;
  typename BasicGraph<VertexType>::basic_iterator end_;
  VertexType* vertex_;
  std::function<bool(VertexType* const&)> filter_;
};

#endif  // EDGE_LIST_HPP_