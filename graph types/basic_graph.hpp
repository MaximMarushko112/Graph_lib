#ifndef BASIC_GRAPH_HPP_
#define BASIC_GRAPH_HPP_

#include "../utils/edge_hash.hpp"

template<typename VertexType>
class basic_graph {
 public:
  basic_graph() = default;

  basic_graph(const basic_graph& g) : vertexes_set_(g.vertexes_set_), edges_set_(g.edges_set_) {}

  basic_graph(basic_graph&& g) : vertexes_set_(std::move(g.vertexes_set_)), edges_set_(std::move(g.edges_set_)) {}

  basic_graph& operator=(const basic_graph& g) {
    if (&g != this) {
      vertexes_set_ = g.vertexes_set_;
      edges_set_ = g.edges_set_;
    }
    return *this;
  }

  basic_graph& operator=(basic_graph&& g) {
    if (&g != this) {
      vertexes_set_ = std::move(g.vertexes_set_);
      edges_set_ = std::move(g.edges_set_);
    }
    return *this;
  }

  bool vertex_in_graph(VertexType* const & vertex) const { return vertexes_set_.find(vertex) != vertexes_set_.end(); }

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

  auto vertexes_begin() const { return vertexes_set_.begin(); }

  auto vertexes_end() const { return vertexes_set_.end(); }

  auto edges_begin() const { return edges_set_.begin(); }

  auto edges_end() const { return edges_set_.end(); }

 private:
  std::unordered_set<VertexType*> vertexes_set_;
  std::unordered_set<std::pair<VertexType*, VertexType*>, EdgeHash<VertexType>> edges_set_;
};

#endif // BASIC_GRAPH_HPP_