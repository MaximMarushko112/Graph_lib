#ifndef BASIC_GRAPH_HPP_
#define BASIC_GRAPH_HPP_

#include <functional>
#include <iterator>

#include "../utils/edge_hash.hpp"

template<typename VertexType>
class BasicGraph {
 private:
  template <bool IsConst>
  class VertexIterator;

 public:
  using basic_iterator               = VertexIterator<false>;
  using const_basic_iterator         = VertexIterator<true>;
  using reverse_basic_iterator       = std::reverse_iterator<basic_iterator>;
  using const_reverse_basic_iterator = std::reverse_iterator<const_basic_iterator>;
  using difference_type              = std::ptrdiff_t;
  
  BasicGraph() = default;

  BasicGraph(const BasicGraph& g) : vertexes_set_(g.vertexes_set_), edges_set_(g.edges_set_) {}

  BasicGraph(BasicGraph&& g) : vertexes_set_(std::move(g.vertexes_set_)), edges_set_(std::move(g.edges_set_)) {}

  BasicGraph& operator=(const BasicGraph& g) {
    if (&g != this) {
      vertexes_set_ = g.vertexes_set_;
      edges_set_ = g.edges_set_;
    }
    return *this;
  }

  BasicGraph& operator=(BasicGraph&& g) {
    if (&g != this) {
      vertexes_set_ = std::move(g.vertexes_set_);
      edges_set_ = std::move(g.edges_set_);
    }
    return *this;
  }

  bool vertex_in_graph(VertexType* const & vertex) const { return vertexes_set_.find(vertex) != vertexes_set_.end(); }

  bool edge_in_graph(VertexType* const & first, VertexType* const & second) const { return edges_set_.find({first, second}) != edges_set_.end(); }

  bool add_vertex(VertexType* const & vertex) { return vertexes_set_.insert(vertex).second; }

  bool remove_vertex(VertexType* const & vertex) {
    auto vertex_iterator = vertexes_set_.find(vertex); 
    if (vertex_iterator == vertexes_set_.end())
      throw std::invalid_argument("There is no such vertex in graph");
    
    vertexes_set_.erase(vertex_iterator);
    return true;
  }

  bool add_edge(VertexType* const & first, VertexType* const & second, bool directed) {  
    if (!vertex_in_graph(first) || !vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");
    edges_set_.insert({first, second});
    if (!directed)
      edges_set_.insert({second, first});
    return true;
  }

  bool remove_edge(VertexType* const & first, VertexType* const & second, bool directed) {
    if (!vertex_in_graph(first) || !vertex_in_graph(second))
      throw std::invalid_argument("There are no such vertexes in graph");               
    edges_set_.erase({first, second});
    if (!directed)
      edges_set_.erase({second, first});
  }

  basic_iterator vertexes_begin(std::function<bool(VertexType* const &)> filter = ret_true) const { 
    return basic_iterator(vertexes_set_.begin(), vertexes_set_.end(), filter); 
  }

  basic_iterator vertexes_end(std::function<bool(VertexType* const &)> filter = ret_true) const { 
    return basic_iterator(vertexes_set_.end(), vertexes_set_.end(), filter); 
  }

  auto edges_begin() const { return edges_set_.begin(); }

  auto edges_end() const { return edges_set_.end(); }

 private:
  static bool ret_true(VertexType* const &) { return true; }
  
  std::unordered_set<VertexType*> vertexes_set_;
  std::unordered_set<std::pair<VertexType*, VertexType*>, EdgeHash<VertexType>> edges_set_;
};

template<typename VertexType>
template<bool IsConst>
class BasicGraph<VertexType>::VertexIterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type        = std::conditional_t<IsConst, const VertexType*, VertexType*>;
  using reference         = std::conditional_t<IsConst, const VertexType* const &, VertexType* const &>;
  using pointer           = std::conditional_t<IsConst, const VertexType*, VertexType*>;

  VertexIterator(typename std::unordered_set<VertexType*>::const_iterator vertex, typename std::unordered_set<VertexType*>::const_iterator end, 
           std::function<bool(VertexType* const &)> filter) : vertex_(vertex), end_(end), filter_(filter) {
    while (vertex_ != end_ && !filter_(*vertex_))
      ++vertex;
  }

  VertexIterator& operator++() {
    ++vertex_;
    while (vertex_ != end_ && !filter_(*vertex_)) 
      ++vertex_;
    return *this;
  }

  VertexIterator operator++(int) {
    VertexIterator copy(*this);
      ++copy;
    return copy;
  }

  reference operator*() const {
    return *vertex_;
  }

  pointer operator->() const {
    return *vertex_;
  }

  bool operator==(const VertexIterator& other) const {
    return vertex_ == other.vertex_;
  }
  
  bool operator!=(const VertexIterator& other) const {
    return vertex_ != other.vertex_;
  }

 private:
  typename std::unordered_set<VertexType*>::const_iterator vertex_;
  typename std::unordered_set<VertexType*>::const_iterator end_;
  std::function<bool(VertexType* const &)> filter_;
};

#endif // BASIC_GRAPH_HPP_