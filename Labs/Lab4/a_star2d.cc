#include <queue>
#include <cmath>
#include <algorithm>

#include "a_star2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;

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
    //Returns true if the NodeWrapper pointed to by nwPtr is found among
    //those pointed to by the elements of nwPtrVec; otherwise returns false
    bool is_present(const NodeWrapperPtr nwPtr,
                    const std::vector<NodeWrapperPtr>& nwPtrVec){
      for(auto n : nwPtrVec){
        if(*n == *nwPtr)
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

    ///////////////////////////////////////////////////////////////////
    // EXAMPLE HEURISTIC FUNCTION
    // YOU WILL NEED TO MODIFY THIS OR WRITE YOUR OWN FUNCTION
    ///////////////////////////////////////////////////////////////////
    double Heuristic(
        const std::shared_ptr<Node2D>& current_ptr,
        const std::shared_ptr<Node2D>& end_ptr) {
        double dx = abs(end_ptr->Data().x()-current_ptr->Data().x());
        double dy = abs(end_ptr->Data().y()-current_ptr->Data().y());
      //return sqrt(pow(dx,2)+pow(dy,2)); //admissible 1, euclidean distance
      //return dx + dy; //overestimate
      return (dx+dy) + (sqrt(2)-2)*std::min(dx,dy); //admissible 2, diagonal distance
      //return 0; //zero heuristic
    }

  }

  PathInfo AStar2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

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

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    // SOME EXAMPLE CODE INCLUDED BELOW
    ///////////////////////////////////////////////////////////////////
    NodeWrapperPtr node_to_explore;
    PathInfo path_info;

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    nw_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(nw_ptr);

    while(true){
      if(to_explore.empty()){
        std::cout << " No path to destination found." <<std::endl;
        return path_info;
      }
      node_to_explore = to_explore.top();to_explore.pop();
      if(is_present(node_to_explore,explored)){
        continue;
      }
      if(*node_to_explore->node_ptr == *end_ptr){
        break;
      }
      else{
        explored.push_back(node_to_explore);
        auto edges = graph.Edges(node_to_explore->node_ptr);
        for(auto edge : edges){
          auto& neighbor = edge.Sink();
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->parent = node_to_explore;
          nw_ptr->node_ptr = neighbor;
          nw_ptr->cost = node_to_explore->cost + edge.Cost();
          nw_ptr->heuristic = Heuristic(neighbor,end_ptr);
          to_explore.push(nw_ptr);
        }
      }
    }

    // Create a PathInfo
    path_info.details.num_nodes_explored = explored.size();
    path_info.details.path_cost = node_to_explore->cost;
    path_info.path.push_back(node_to_explore->node_ptr);
    path_info.details.path_length = 1;
    while(!(*node_to_explore->node_ptr == *start_ptr)){
      node_to_explore = node_to_explore->parent;
      path_info.path.push_back(node_to_explore->node_ptr);
      path_info.details.path_length += 1;
    }

    std::reverse(path_info.path.begin(),path_info.path.end());
    path_info.details.run_time = timer.Stop();

    // You must return a PathInfo
    return path_info;
  }
  
}
