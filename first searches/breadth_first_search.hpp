#ifndef BREADTH_FIRST_SEARCH_HPP_
#define BREADTH_FIRST_SEARCH_HPP_

#include <queue>
#include <unordered_map>
#include <vector>

enum class Colour {
  White,
  Gray,
  Black
};

template <typename Graph>
class BFSVisitor {
 public:
  void initialize_vertex(typename Grapg::vertex_descriptor& vertex, Graph& graph) = 0;

  void initialize_vertex(typename Grapg::vertex_descriptor& vertex, Graph& graph) = 0;

  void discover_vertex  (typename Grapg::vertex_descriptor& vertex, Graph& graph) = 0;

  void examine_vertex   (typename Grapg::vertex_descriptor& vertex, Graph& graph) = 0;

  void finish_vertex    (typename Grapg::vertex_descriptor& vertex, Graph& graph) = 0;

  void examine_edge     (typename Grapg::vertex_descriptor& start, typename Grapg::vertex_descriptor& finish, Graph& graph) = 0;

  void tree_edge        (typename Grapg::vertex_descriptor& start, typename Grapg::vertex_descriptor& finish, Graph& graph) = 0;

  void non_tree_edge    (typename Grapg::vertex_descriptor& start, typename Grapg::vertex_descriptor& finish, Graph& graph) = 0;

  void gray_target      (typename Grapg::vertex_descriptor& start, typename Grapg::vertex_descriptor& finish, Graph& graph) = 0;

  void black_target     (typename Grapg::vertex_descriptor& start, typename Grapg::vertex_descriptor& finish, Graph& graph) = 0;
};

template <typename Graph, typename Visitor>
void breadth_first_search(const Graph& g, std::vector<typename Graph::vertex_descriptor> roots, Visitor visitor, 
                          std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  std::queue<typename Graph::vertex_descriptor> queue;
  for (auto vertex = g.vertexes_begin(); vertex != g.vertexes_end(); ++vertex) {
    Visitor.initialize_vertex(*vertex, g);
    colours[*vertex] = White;
  }
  
  for (auto root = roots.begin(); root != roots.end(); ++root) {
    colours[*root] = Gray;
    visitor.discover_vertex(*root, g);
    queue.push(*root);
  }
  
  while (!queue.empty()) {
    auto vertex = queue.top();
    queue.pop();
    visitor.examine_vertex(vertex, g);
    
    for (auto neighbour = g.neighbours_begin(vertex); neighbour != g.neighbours_end(vertex); ++neighbour) {
      visitor.examine_edge(vertex, *neighbour, g);
      Colour neighbour_colour = colours[*neighbour];
      if (neighbour_colour == White) {
        visitor.tree_edge(vertex, *neighbour, g);
        colours[*neighbour] = Gray;
        visitor.discover_vertex(*neighbour, g);
        queue.push(*neighbour);
      }
      else {
        visitor.non_tree_edge(vertex, *neighbour, g);
        if (neighbour_color == Gray) 
          visitor.gray_target(vertex, *neighbour, g);
        else 
          visitor.black_target(vertex, *neighbour, g);
      }
    }
    
    colour[vertex] = Black;
    visitor.finish_vertex(vertex, g);
  }
}

template <typename Graph, typename Visitor>
void breadth_first_search(const Graph& g, typename Graph::vertex_descriptor root, Visitor visitor, 
                          std::unordered_map<typename Graph::vertex_descriptor, Colour>& colours) {
  breadth_first_search(g, std::vector<typename Graph::vertex_descriptor>(1, root), visitor, colours);
}

#endif // BREADTH_FIRST_SEARCH_HPP_