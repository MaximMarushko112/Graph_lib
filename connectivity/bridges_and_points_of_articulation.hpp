#ifndef BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_
#define BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "../first searches/depth_first_search.hpp"
#include "../utils/edge_hash.hpp"

template <typename Graph>
class BridgeVisitor : public DFSVisitor<Graph> {
 public:
  BridgeVisitor(std::unordered_set<std::pair<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor>, typename Graph::edge_hash>* const & bridges) 
  : bridges_(bridges), timer_(0) {}

  void discover_vertex (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    ++timer_;
    tin_[vertex] = timer_;
    fup_[vertex] = timer_;
  }

  void finish_vertex (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    std::size_t tin_v = tin_[vertex];
    std::size_t fup_v = fup_[vertex];

    for (auto tree_neighbour = tree_edges_[vertex].begin(); tree_neighbour != tree_edges_[vertex].end(); ++tree_neighbour) 
      fup_v = std::min<std::size_t>(fup_v, fup_[*tree_neighbour]);
    fup_[vertex] = fup_v;
    
    for (auto neighbour = graph.neighbours_begin(vertex); neighbour != graph.neighbours_end(vertex); ++neighbour) {
      if (fup_[*neighbour] > tin_v) 
        bridges_->insert({vertex, *neighbour});
    }
  }

  void tree_edge (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {
    parents_[finish] = start;
    tree_edges_[start].insert(finish);
  }

  void back_edge (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {
    if (finish != parents_[start])
      fup_[start] = std::min<std::size_t>(fup_[start], tin_[finish]);
  }

 private:
  std::size_t timer_;
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t> tin_;
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t> fup_;
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor> parents_;
  std::unordered_map<typename Graph::vertex_descriptor, std::unordered_set<typename Graph::vertex_descriptor>> tree_edges_;
  std::unordered_set<std::pair<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor>, typename Graph::edge_hash>* bridges_;
};

template<typename Graph>
auto FindBridges(const Graph& graph) {
  std::unordered_set<std::pair<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor>, typename Graph::edge_hash> bridges;
  std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
  DepthFirstSearch(graph, *graph.vertexes_begin(), BridgeVisitor<Graph>(&bridges), colours);
  return bridges;
}

template <typename Graph>
class PointsOfArticulationVisitor : public DFSVisitor<Graph> {
 public:
  PointsOfArticulationVisitor(std::unordered_set<typename Graph::vertex_descriptor>* const & points_of_articulation) 
  : points_of_articulation_(points_of_articulation), timer_(0), start_recursive_calls_count_(0) {}

  void start_vertex (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    start_ = vertex;
  }

  void discover_vertex (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    ++timer_;
    tin_[vertex] = timer_;
    fup_[vertex] = timer_;
  }

  void finish_vertex (const typename Graph::vertex_descriptor& vertex, const Graph& graph) {
    std::size_t tin_v = tin_[vertex];
    std::size_t fup_v = fup_[vertex];

    if (vertex == start_)
      return;

    for (auto tree_neighbour = tree_edges_[vertex].begin(); tree_neighbour != tree_edges_[vertex].end(); ++tree_neighbour) 
      fup_v = std::min<std::size_t>(fup_v, fup_[*tree_neighbour]);
    fup_[vertex] = fup_v;
    
    bool is_point_of_articulation = true;
    for (auto neighbour = graph.neighbours_begin(vertex); neighbour != graph.neighbours_end(vertex); ++neighbour) {
      if (*neighbour != parents_[vertex] && fup_[*neighbour] < tin_v) {
        is_point_of_articulation = false;
        break;
      }
    }
    if (is_point_of_articulation)
      points_of_articulation_->insert(vertex);
  }

  void tree_edge (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {
    parents_[finish] = start;
    tree_edges_[start].insert(finish);
    if (start == start_) {
      ++start_recursive_calls_count_;
      if (start_recursive_calls_count_ > 1) {
        points_of_articulation_->insert(start);
      }
    }
  }

  void back_edge (const typename Graph::vertex_descriptor& start, const typename Graph::vertex_descriptor& finish, const Graph& graph) {
    if (finish != parents_[start])
      fup_[start] = std::min<std::size_t>(fup_[start], tin_[finish]);
  }

 private:
  std::size_t timer_;
  typename Graph::vertex_descriptor start_;
  std::size_t start_recursive_calls_count_;
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t> tin_;
  std::unordered_map<typename Graph::vertex_descriptor, std::size_t> fup_;
  std::unordered_map<typename Graph::vertex_descriptor, typename Graph::vertex_descriptor> parents_;
  std::unordered_map<typename Graph::vertex_descriptor, std::unordered_set<typename Graph::vertex_descriptor>> tree_edges_;
  std::unordered_set<typename Graph::vertex_descriptor>* points_of_articulation_;
};

template<typename Graph>
auto FindPointsOfArticulation(const Graph& graph) {
  std::unordered_set<typename Graph::vertex_descriptor> points_of_articulation;
  std::unordered_map<typename Graph::vertex_descriptor, Colour> colours;
  DepthFirstSearch(graph, *graph.vertexes_begin(), PointsOfArticulationVisitor<Graph>(&points_of_articulation), colours);
  return points_of_articulation;
}

#endif // BRIDGES_AND_POINTS_OF_ARTICULATION_HPP_