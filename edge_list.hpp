#ifndef EDGE_LIST_HPP_
#define EDGE_LIST_HPP_

#include <functional>
#include <iterator>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

template<typename VertexType, bool Directed, bool Weighted=false, typename WeightType=int>
class edge_list {
 private:  
  struct EdgeHash {
    std::size_t operator()(const std::pair<VertexType*, VertexType*>& pair) const noexcept{
      std::size_t h1 = std::hash<VertexType*>{}(pair.first);
      std::size_t h2 = std::hash<VertexType*>{}(pair.second);
      return h1 ^ (h2 << 1);
    }
  };
  template <bool IsConst>
  class Iterator;

 public:
  using iterator                = Iterator<false>;
  using const_iterator          = Iterator<true>;
  using reverse_iterator        = std::reverse_iterator<iterator>;
  using const_reverse_iterator  = std::reverse_iterator<const_iterator>;
  using difference_type         = std::ptrdiff_t;
  
  edge_list() = default;

  edge_list(const edge_list& g) : vertexes_(g.vertexes_), edges_(g.edges_) {}

  edge_list(edge_list&& g) : vertexes_(std::move(g.vertexes_)), edges_(std::move(g.edges_)) {}

  edge_list& operator=(const edge_list& g) {
    if (&g != this) {
      vertexes_ = g.vertexes_;
      edges_ = g.edges_;
    }
    return *this;
  }

  edge_list& operator=(edge_list&& g) {
    if (&g != this) {
      vertexes_ = std::move(g.vertexes_);
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
    return vertexes_.insert(vertex).second;
  }

  bool remove_vertex(VertexType* const & vertex) {  
    auto vertex_iterator = vertexes_.find(vertex); 
    if (vertex_iterator == vertexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    
    for (auto neighbour : vertexes_) {
      remove_edge(vertex, neighbour);
      remove_edge(neighbour, vertex);
    }

    vertexes_.erase(vertex_iterator);
    return true;
  }

  bool add_edge(VertexType* const & first, VertexType* const & second) {  
    if (vertexes_.find(first) == vertexes_.end() || vertexes_.find(second) == vertexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");
    
    if constexpr (Weighted)
      return false;
    else {
      return edges_.insert({{first, second}, 1}).second;
    }
    return true;
  }

  bool add_edge(VertexType* const & first, VertexType* const & second, const WeightType& weight) { 
    if (vertexes_.find(first) == vertexes_.end() || vertexes_.find(second) == vertexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");
    
    if constexpr (!Weighted)
      return false;
    else {
      return edges_.insert({{first, second}, weight}).second;
    }
    return true;
  }

  bool remove_edge(VertexType* const & first, VertexType* const & second) {
    if (vertexes_.find(first) == vertexes_.end() || vertexes_.find(second) == vertexes_.end())
      throw std::invalid_argument("There are no such vertexes in graph");               
    
    auto edge = edges_.find(std::pair{first, second});
    if (edge != edges_.end())
      edges_.erase(edge);
    else if (Directed)
      return false;
    
    if (!Directed) {
      auto reverse_edge = edges_.find(std::pair{second, first});
      if (reverse_edge != edges_.end())
        edges_.erase(edge);
    }

    return true;
  }

  iterator neighbours_begin(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) {
    auto vertex_iter = vertexes_.find(vertex);
    if (vertex_iter == vertexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, vertexes_.begin(), vertexes_.end(), vertex, filter);
  }

  iterator neighbours_end(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) {
    auto vertex_iter = vertexes_.find(vertex);
    if (vertex_iter == vertexes_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(&edges_, vertexes_.end(), vertexes_.end(), vertex, filter);
  }

 private:
  static bool ret_true(VertexType* const &) { return true; }

  std::unordered_set<VertexType*> vertexes_;
  std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash> edges_;
};

template <typename VertexType, bool Directed, bool Weighted, typename WeightType>
template <bool IsConst>
class edge_list<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type        = std::conditional_t<IsConst, const VertexType, VertexType>;
  using reference         = std::conditional_t<IsConst, const VertexType&, VertexType&>;
  using pointer           = std::conditional_t<IsConst, const VertexType*, VertexType*>;

  Iterator(std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash>* edges, typename std::unordered_set<VertexType*>::iterator neighbour, 
           typename std::unordered_set<VertexType*>::iterator end, VertexType* vertex, std::function<bool(VertexType* const &)> filter) 
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
    return **neighbour_;
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
  std::unordered_map<std::pair<VertexType*, VertexType*>, WeightType, EdgeHash>* edges_;
  typename std::unordered_set<VertexType*>::iterator neighbour_;
  typename std::unordered_set<VertexType*>::iterator end_;
  VertexType* vertex_;
  std::function<bool(VertexType* const &)> filter_;
};

#endif // EDGE_LIST_HPP_