#ifndef DEPTH_FIRST_SEARCH_HPP_
#define DEPTH_FIRST_SEARCH_HPP_

#include <unordered_map>
#include <vector>

#include "colours.hpp"

template <typename Graph>
class DFSVisitor {
 public:
  DFSVisitor() = default;

  void initialize_vertex    (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void start_vertex         (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void discover_vertex      (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void examine_vertex       (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void finish_vertex        (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void examine_edge         (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void tree_edge            (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void back_edge            (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void forward_or_cross_edge(const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void finish_edge          (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}
};

template <typename Graph, typename Visitor>
void depth_first_search(const Graph& g, typename Graph::vertex_descriptor root, Visitor visitor, 
                          std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    visitor.initialize_vertex(*vertex, g);
    colours[*vertex] = Colour::White;
  }

  visitor.start_vertex(root, g);
  depth_first_visit(g, root, visitor, colours);
  
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    if (colours[*vertex] == Colour::White) {
      visitor.start_vertex(*vertex, g);
      depth_first_visit(g, *vertex, visitor, colours);
    }
  }
}

template <typename Graph, typename Visitor>
void depth_first_visit(const Graph& g, typename Graph::vertex_descriptor root, Visitor& visitor, 
                       std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  colours[root] = Colour::Gray;
  visitor.discover_vertex(root, g);

  for (auto neighbour = g.neighbours_begin(root); neighbour != g.neighbours_end(root); ++neighbour) {
    visitor.examine_edge(root, *neighbour, g);
    Colour neighbour_colour = colours[*neighbour];
    if (neighbour_colour == Colour::White) {
      visitor.tree_edge(root, *neighbour, g);
      depth_first_visit(g, *neighbour, visitor, colours);
    }
    else if (neighbour_colour == Colour::Gray) {
      visitor.back_edge(root, *neighbour, g);
    }
    else {
      visitor.forward_or_cross_edge(root, *neighbour, g);
    }
    visitor.finish_edge(root, *neighbour, g);
  }
  colours[root] = Colour::Black;
  visitor.finish_vertex(root, g);
}

#endif // DEPTH_FIRST_SEARCH_HPP_