#ifndef EDGE_HASH_HPP_
#define EDGE_HASH_HPP_

#include <utility>

template<typename VertexType>
struct EdgeHash {
  std::size_t operator()(const std::pair<VertexType*, VertexType*>& pair) const noexcept{
    std::size_t h1 = std::hash<VertexType*>{}(pair.first);
    std::size_t h2 = std::hash<VertexType*>{}(pair.second);
    return h1 ^ (h2 << 1);
  }
};

#endif // EDGE_HASH_HPP_