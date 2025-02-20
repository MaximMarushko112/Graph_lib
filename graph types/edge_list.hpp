#ifndef EDGE_LIST_HPP_
#define EDGE_LIST_HPP_

#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "basic_graph.hpp"
#include "../utils/edge_hash.hpp"

template<typename VertexType, bool Directed, bool Weighted=false, typename WeightType=int>
class edge_list : public basic_graph<VertexType> {
 private:  
  template <bool IsConst>
  class Iterator;

 public:
  using vertex_descriptor       = VertexType*;
  using weight                  = WeightType;
  using edge_hash               = EdgeHash<VertexType>;
  using iterator                = Iterator<false>;
  using const_iterator          = Iterator<true>;
  using reverse_iterator        = std::reverse_iterator<iterator>;
  using const_reverse_iterator  = std::reverse_iterator<const_iterator>;
  using difference_type         = std::ptrdiff_t;
  
  edge_list() = default;

  edge_list(const edge_list& g) : basic_graph<VertexType>(g), edges_(g.edges_) {}

  edge_list(edge_list&& g) : basic_graph<VertexType>(g), edges_(std::move(g.edges_)) {}

  edge_list& operator=(const edge_list& g) {
    if (&g != this) {
      basic_graph<VertexType>(g);
      edges_ = g.edges_;
    }
    return *this;
  }

  edge_list& operator=(edge_list&& g) {
    if (&g != this) {
      basic_graph<VertexType>(g);
      edges_ = std::move(g.edges_);
    }
    return *this;
  }

  void swap(edge_list& g) {
    edge_list tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  bool add_vertex(VertexType* const & vertex) {
    return basic_graph<VertexType>::add_vertex(vertex);
  }

  bool remove_vertex(VertexType* const & vertex) {  
    if (!basic_graph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    
    for (auto neighbour : basic_graph<VertexType>::vertexes_set_) {
      remove_edge(vertex, neighbour);
      remove_edge(neighbour, vertex);
    }

    return basic_graph<VertexType>::remove_vertex(vertex);
  }

  bool add_edge(VertexType* const & first, VertexType* const & second) {  
    if constexpr (Weighted)
      return false;
    else {
      basic_graph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, 1}).second;
    }
    return true;
  }

  bool add_edge(VertexType* const & first, VertexType* const & second, const WeightType& weight) { 
    if constexpr (!Weighted)
      return false;
    else {
      basic_graph<VertexType>::add_edge(first, second, Directed);
      return edges_.insert({{first, second}, weight}).second;
    }
    return true;
  }

  bool remove_edge(VertexType* const & first, VertexType* const & second) {
    basic_graph<VertexType>::remove_edge(first, second, Directed);
    auto edge = edges_.find(std::pair{first, second});
    bool edge_found = edge != edges_.end();
    if (edge != edges_.end())
      edges_.erase(edge);
    else if (Directed)
      return false;
    
    if (!Directed) {
      auto reverse_edge = edges_.find(std::pair{second, first});
      if (!edge_found)
        edge_found = reverse_edge != edges_.end();
      if (reverse_edge != edges_.end())
        edges_.erase(edge);
    }

    return edge_found;
  }

  WeightType edge_weight(VertexType* const & first, VertexType* const & second) const {
    if (!basic_graph<VertexType>::vertex_in_graph(first) || !basic_graph<VertexType>::vertex_in_graph(second))
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

  iterator neighbours_begin(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) const {
    if (!basic_graph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, basic_graph<VertexType>::vertexes_begin(), basic_graph<VertexType>::vertexes_end(), vertex, filter);
  }

  iterator neighbours_end(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) const {
    if (!basic_graph<VertexType>::vertex_in_graph(vertex))
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, basic_graph<VertexType>::vertexes_end(), basic_graph<VertexType>::vertexes_end(), vertex, filter);
  }

 private:
  static bool ret_true(VertexType* const &) { return true; }

  std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash<VertexType>> edges_;
};

template <typename VertexType, bool Directed, bool Weighted, typename WeightType>
template <bool IsConst>
class edge_list<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type        = std::conditional_t<IsConst, const VertexType*, VertexType*>;
  using reference         = std::conditional_t<IsConst, const VertexType* const &, VertexType* const &>;
  using pointer           = std::conditional_t<IsConst, const VertexType*, VertexType*>;

  Iterator(const std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash<VertexType>>* edges, typename std::unordered_set<VertexType*>::const_iterator neighbour, 
           typename std::unordered_set<VertexType*>::const_iterator end, VertexType* vertex, std::function<bool(VertexType* const &)> filter) 
           : edges_(edges), neighbour_(neighbour), end_(end), vertex_(vertex), filter_(filter) {
    while (neighbour_ != end_ && (!filter_(*neighbour_) || (edges_->find({vertex_, *neighbour_}) == edges_->end() && edges_->find({*neighbour_, vertex_}) == edges_->end()))) 
      ++neighbour_;
  }

  Iterator& operator++() {
    ++neighbour_;
    while (neighbour_ != end_ && ((edges_->find({vertex_, *neighbour_}) == edges_->end() && edges_->find({*neighbour_, vertex_}) == edges_->end()) || !filter_(*neighbour_))) 
      ++neighbour_;
    return *this;
  }

  Iterator operator++(int) {
    Iterator copy(*this);
      ++copy;
    return copy;
  }

  reference operator*() const {
    return *neighbour_;
  }

  pointer operator->() const {
    return *neighbour_;
  }

  bool operator==(const Iterator& other) const {
    return neighbour_ == other.neighbour_ && vertex_ == other.vertex_;
  }
  
  bool operator!=(const Iterator& other) const {
    return neighbour_ != other.neighbour_;
  }

 private:
  const std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash<VertexType>>* edges_;
  typename std::unordered_set<VertexType*>::const_iterator neighbour_;
  typename std::unordered_set<VertexType*>::const_iterator end_;
  VertexType* vertex_;
  std::function<bool(VertexType* const &)> filter_;
};

#endif // EDGE_LIST_HPP_