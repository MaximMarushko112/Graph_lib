#ifndef ADJACENCY_LIST_HPP_
#define ADJACENCY_LIST_HPP_

#include <functional>
#include <iterator>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

template<typename VertexType, bool Directed, bool Weighted=false, typename WeightType=int>
class adjacency_list {
 private:
  template <bool IsConst>
  class Iterator;

 public:
  using iterator                = Iterator<false>;
  using const_iterator          = Iterator<true>;
  using reverse_iterator        = std::reverse_iterator<iterator>;
  using const_reverse_iterator  = std::reverse_iterator<const_iterator>;
  using difference_type         = std::ptrdiff_t;
  
  adjacency_list() = default;

  adjacency_list(const adjacency_list& g) : edges_(g.edges_) {}

  adjacency_list(adjacency_list&& g) : edges_(std::move(g.edges_)) {}

  adjacency_list& operator=(const adjacency_list& g) {
    if (&g != this) 
      edges_ = g.edges_;
    return *this;
  }

  adjacency_list& operator=(adjacency_list&& g) {
    if (&g != this)
      edges_ = std::move(g.edges_);
    return *this;
  }

  void swap(adjacency_list& g) {
    adjacency_list tmp(std::move(g));
    g = *this;
    *this = tmp;
  }

  bool add_vertex(VertexType* const & vertex) {
    vertexes.insert(vertex);
    return edges_.insert({vertex, std::unordered_map<VertexType*, WeightType>()}).second;
  }

  bool remove_vertex(VertexType* const & vertex) {  
    auto vertex_iterator = edges_.find(vertex);   
    if (vertex_iterator == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    
    for (auto other_vertex : edges_) {
      remove_edge(vertex, other_vertex.first);
      if (Directed)  
        remove_edge(other_vertex.first, vertex);  
    }

    vertexes.erase(vertexes.find(vertex));
    edges_.erase(vertex_iterator);
    return true;
  }

  bool add_edge(VertexType* const & first, VertexType* const & second) {  
    auto first_vertex_adjacency  = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);   
    if (first_vertex_adjacency == edges_.end() || second_vertex_adjacency == edges_.end())
      throw std::invalid_argument("There are no such vertexes in graph");
    
    if constexpr (Weighted)
      return false;
    else {
      first_vertex_adjacency->second.insert({second, 1});
      if (!Directed) {
        second_vertex_adjacency->second.insert({first, 1});
      }
    }
    return true;
  }
  
  bool add_edge(VertexType* const & first, VertexType* const & second, const WeightType& weight) { 
    auto first_vertex_adjacency  = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);   
    if (first_vertex_adjacency == edges_.end() || second_vertex_adjacency == edges_.end()) 
      throw std::invalid_argument("There are no such vertexes in graph");
    if constexpr (!Weighted)
      return false;               
    else {
      first_vertex_adjacency->second.insert({second, weight}); 
      if (!Directed) {
        second_vertex_adjacency->second.insert({first, weight});
      }
    }
    return true;
  }
  
  bool remove_edge(VertexType* const & first, VertexType* const & second) {
    auto first_vertex_adjacency  = edges_.find(first);
    auto second_vertex_adjacency = edges_.find(second);   
    if (first_vertex_adjacency == edges_.end() || second_vertex_adjacency == edges_.end())
      throw std::invalid_argument("There are no such vertexes in graph");               
    
    auto edge = first_vertex_adjacency->second.find(second);
    if (edge == first_vertex_adjacency->second.end()) 
      return false;
    first_vertex_adjacency->second.erase(edge);

    if (!Directed) {
      auto reverse_edge = second_vertex_adjacency->second.find(first);
      if (reverse_edge == second_vertex_adjacency->second.end()) 
        return false;
      second_vertex_adjacency->second.erase(reverse_edge);
    }

    return true;
  }

  iterator neighbours_begin(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) {
    auto vertex_iter = edges_.find(vertex);
    if (vertex_iter == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(vertex_iter->second.begin(), vertex_iter->second.end(), filter);
  }

  iterator neighbours_end(VertexType* const & vertex, std::function<bool(VertexType* const &)> filter = ret_true) {
    auto vertex_iter = edges_.find(vertex);
    if (vertex_iter == edges_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    return iterator(vertex_iter->second.end(), vertex_iter->second.end(), filter);
  }

 private:
  static bool ret_true(VertexType* const &) { return true; }

 public:
  std::unordered_set<VertexType*> vertexes;
 
 private:
  std::unordered_map<VertexType*, std::unordered_map<VertexType*, WeightType>> edges_;
};

template <typename VertexType, bool Directed, bool Weighted, typename WeightType>
template <bool IsConst>
class adjacency_list<VertexType, Directed, Weighted, WeightType>::Iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type        = std::conditional_t<IsConst, const VertexType, VertexType>;
  using reference         = std::conditional_t<IsConst, const VertexType&, VertexType&>;
  using pointer           = std::conditional_t<IsConst, const VertexType*, VertexType*>;

  Iterator(typename std::unordered_map<VertexType*, WeightType>::iterator neighbour_iter, 
           typename std::unordered_map<VertexType*, WeightType>::iterator end, std::function<bool(VertexType* const &)> filter) 
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

  reference operator*() const {
    return *neighbour_iter_->first;
  }

  pointer operator->() const {
    return neighbour_iter_->first;
  }

  bool operator==(const Iterator& other) const {
    return neighbour_iter_ == other.neighbour_iter_;
  }
  
  bool operator!=(const Iterator& other) const {
    return neighbour_iter_ != other.neighbour_iter_;
  }

 private:
  typename std::unordered_map<VertexType*, WeightType>::iterator neighbour_iter_;
  typename std::unordered_map<VertexType*, WeightType>::iterator end_;
  std::function<bool(VertexType* const &)> filter_;
};

#endif // ADJACENCY_LIST_HPP_