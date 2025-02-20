#ifndef BREADTH_FIRST_SEARCH_HPP_
#define BREADTH_FIRST_SEARCH_HPP_

#include <queue>
#include <unordered_map>
#include <vector>

#include "colours.hpp"

template <typename Graph>
class BFSVisitor {
 public:
  BFSVisitor() = default;

  void initialize_vertex(const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void discover_vertex  (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void examine_vertex   (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void finish_vertex    (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {}

  void examine_edge     (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void tree_edge        (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void non_tree_edge    (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void gray_target      (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}

  void black_target     (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {}
};

template <typename Graph, typename Visitor>
void breadth_first_search(const Graph& g, std::vector<typename Graph::vertex_descriptor> const & roots, Visitor visitor, 
                          std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  std::queue<typename Graph::vertex_descriptor> queue;
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    visitor.initialize_vertex(*vertex, g);
    colours[*vertex] = Colour::White;
  }
  
  for (auto root = roots.begin(); root != roots.end(); ++root) {
    colours[*root] = Colour::Gray;
    visitor.discover_vertex(*root, g);
    queue.push(*root);
  }
  
  while (!queue.empty()) {
    auto vertex = queue.front();
    queue.pop();
    visitor.examine_vertex(vertex, g);
    
    for (auto neighbour = g.neighbours_begin(vertex); neighbour != g.neighbours_end(vertex); ++neighbour) {
      visitor.examine_edge(vertex, *neighbour, g);
      Colour neighbour_colour = colours[*neighbour];
      if (neighbour_colour == Colour::White) {
        visitor.tree_edge(vertex, *neighbour, g);
        colours[*neighbour] = Colour::Gray;
        visitor.discover_vertex(*neighbour, g);
        queue.push(*neighbour);
      }
      else {
        visitor.non_tree_edge(vertex, *neighbour, g);
        if (neighbour_colour == Colour::Gray) 
          visitor.gray_target(vertex, *neighbour, g);
        else 
          visitor.black_target(vertex, *neighbour, g);
      }
    }
    
    colours[vertex] = Colour::Black;
    visitor.finish_vertex(vertex, g);
  }
}

template <typename Graph, typename Visitor>
void breadth_first_search(const Graph& g, typename Graph::vertex_descriptor root, Visitor visitor, 
                          std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  breadth_first_search(g, std::vector<typename Graph::vertex_descriptor>(1, root), visitor, colours);
}

#endif // BREADTH_FIRST_SEARCH_HPP_