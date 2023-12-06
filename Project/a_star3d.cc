#include <queue>

#include "a_star3d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node3D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    // Returns true if the NodeWrapper pointed to by nwPtr is found among
    // those pointed to by the elements of nwPtrVec: otherwise returns false.
    bool is_present(const NodeWrapperPtr nwPtr,
                    const std::vector<NodeWrapperPtr>& nwPtrVec){
      for (auto n : nwPtrVec) {
        if (*n == *nwPtr) 
          return true;
      }
      return false;
    }

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    double Heuristic(
        const std::shared_ptr<Node3D>& current_ptr,
        const std::shared_ptr<Node3D>& end_ptr) {
          // CORRECT HEURISTIC
          int x_current = current_ptr->Data() [0];
          int y_current = current_ptr->Data() [1];
          int z_current = current_ptr->Data() [2];
          int x_end = end_ptr->Data() [0];
          int y_end = end_ptr->Data() [1];
          int z_end = end_ptr->Data() [2];
          double dist = sqrt(pow(x_current - x_end, 2) + pow(y_current - y_end, 2) + pow(z_current - z_end, 2));
          return dist;
    }

  }

  PathInfo AStar3D::Run(
      const Graph3D& graph, 
      const std::shared_ptr<Node3D> start_ptr, 
      const std::shared_ptr<Node3D> end_ptr) {

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::priority_queue<
      NodeWrapperPtr,
      std::vector<NodeWrapperPtr>,
      std::function<bool(
          const NodeWrapperPtr&, 
          const NodeWrapperPtr& )>> 
        to_explore(NodeWrapperPtrCompare);

    std::vector<NodeWrapperPtr> explored;
    NodeWrapperPtr node_to_explore;
    PathInfo path_info;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES BELOW
    // SOME EXAMPLE CODE PROVIDED
    ///////////////////////////////////////////////////////////////////

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    nw_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(nw_ptr);

    while(true) {

      // Run out of nodes to explore
      if(to_explore.empty()) {
        std::cout << "No path to destination found." <<std::endl;
        return path_info;
      }
    node_to_explore = to_explore.top(); to_explore.pop();

    // Check if we've already explored this node
    if(is_present(node_to_explore, explored)) {
      //std::cout << "Already explored node" << std::endl;
      continue;
    }

    // Check if we've reached the destination
    if(*node_to_explore->node_ptr == *end_ptr) {
      std::cout << "Reached destination" << std::endl;
      break;
    }

    // Otherwise, add the node to the explored list and add its neighbors
    else {
      //std::cout << "Exploring node" << std::endl;
      explored.push_back(node_to_explore);
      auto edges = graph.Edges(node_to_explore->node_ptr);
      for(auto edge : edges) {
        auto& neighbor = edge.Sink();
        NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
        nw_ptr->parent = node_to_explore;
        nw_ptr->node_ptr = neighbor;
        nw_ptr->cost = node_to_explore->cost + edge.Cost();
        nw_ptr->heuristic = Heuristic(neighbor, end_ptr);
        to_explore.push(nw_ptr);
      }
    }
  }

  path_info.details.path_cost = node_to_explore->cost;
  // Note: End node not counted among those explored
  path_info.details.num_nodes_explored = explored.size();
  path_info.path.push_back(node_to_explore->node_ptr);
  path_info.details.path_length = 1;
  while(!(*node_to_explore->node_ptr == *start_ptr)) {
    node_to_explore = node_to_explore->parent;
    path_info.path.push_back(node_to_explore->node_ptr);
    path_info.details.path_length++;
  }
  // Reverse the ordering of the path so it goes from start to end
  std::reverse(path_info.path.begin(), path_info.path.end());
  path_info.details.run_time = timer.Stop();
  return path_info;
  }
  
}


