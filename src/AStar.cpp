#include "AStar.hpp"

/**
 * init the search algorithm with the given inputs
 */
AStar::SearchResult AStar::init(const cv::Mat& image,
          const Pose &start_pose,
          const Pose &end_pose,
          const double &scalar)
{
  image_ = image;
  start_pose_ = start_pose;
  end_pose_ = end_pose;

  total_added_to_queue_ = 0;
  total_popped_from_queue_ = 0;
  scalar_ = scalar;
  successful_solve_ = false;

  if (occupied(start_pose)){
    return START_INSIDE_OBSTACLE;
  }
  if (occupied(end_pose)){
    return END_INSIDE_OBSTACLE;
  }

  // this will set all nodes to use the same scalar
  Node::setHeuristicScalar(scalar);
  // set all nodes to have the same end_pose
  Node::setEndPose(end_pose);

  // initialize possible poses
  initializeMotions();
  return OK;
}

/**
 * Function to generate all possible motions the robot can take
 * 
 * The whole code/motion iteration could've been simpler had I done this differently, but I wanted to make it
 *   seem more extensible for dynamic models
 */
void AStar::initializeMotions()
{
  // move "left" and diagonals
  possible_motions_diagonal_.push_back(Pose(-1, -1));
  possible_motions_cardinal_.push_back(Pose(-1, 0));
  possible_motions_diagonal_.push_back(Pose(-1, 1));

  // move "up/down"
  possible_motions_cardinal_.push_back(Pose(0, -1));
  possible_motions_cardinal_.push_back(Pose(0, 1));

  // move "right" and diagonals
  possible_motions_diagonal_.push_back(Pose(1, -1));
  possible_motions_cardinal_.push_back(Pose(1, 0));
  possible_motions_diagonal_.push_back(Pose(1, 1));
}

/**
 * generate a path from the node N to the start pose
 */
void AStar::nodeToPath(const Node &n, Path &out_path)
{
  out_path.clear();
  Pose current_pose = n.getPosition();
  while (!(current_pose == start_pose_)){
    out_path.push_front(current_pose);
    auto parent = history_map[current_pose].second;;
    current_pose = parent;
  }
}

/**
 * Search for the solution and return the path. If fails, path.empty() should be true
 */
AStar::SearchResult AStar::solve(Path &out_path)
{
  // setup the priority queue
  auto comp = []( Node &a, Node &b ) { return a.getHeuristicCost() > b.getHeuristicCost(); };
  typedef std::priority_queue<Node, std::vector<Node>, decltype(comp)> NodePriorityQueue;

  // priority queue with a custom comparison class that uses the heuristic cost function to sort
  NodePriorityQueue queue(comp);

  // push first node to the queue   
  Node root( start_pose_ );
  queue.push(root);
  ++total_added_to_queue_;
  while(!queue.empty()){
    auto node = queue.top();
    // std::cout << "Popped: " << node.getPosition() << ". Remaining: " << queue.size() << std::endl;
    queue.pop();
    ++total_popped_from_queue_;
    if (node.getPosition() == end_pose_){
      // finished return this path
      std::cout << "Finished! cost = " << node.getCost() << std::endl;
      nodeToPath(node, out_path);
      successful_solve_ = true;
      return FOUND_PATH;
    }

    // try to move in every possible direction from this node. add each to the queue
    //   we can move +/- one in x and y, so we

    // cardinal directions
    auto new_pose_cost = node.getCost() + cardinal_cost;
    for (auto m : possible_motions_cardinal_){
      Pose new_pose = node.getPosition() + m;
      // only valid points
      if (!occupied(new_pose.x, new_pose.y)){
        //  let's see if we have already arrived at this position during the search
        auto it = history_map.find(new_pose);
        if (it == history_map.end()){
          history_map[new_pose] = std::make_pair(new_pose_cost, node.getPosition());
          queue.push(Node(&node, new_pose, new_pose_cost));
          ++total_added_to_queue_;
        }
        else{
          // this is a more efficient path to get to this Pose in the map
          if (new_pose_cost < it->second.first){
            history_map[new_pose].first = new_pose_cost;
            history_map[new_pose].second = node.getPosition();
            queue.push(Node(&node, new_pose, new_pose_cost));
            ++total_added_to_queue_;
          }
        }
      }
    }
    
    // diagonal directions
    new_pose_cost = node.getCost() + diagonal_cost;
    for (auto m : possible_motions_diagonal_){
      Pose new_pose = node.getPosition() + m;
      // only valid points
      if (!occupied(new_pose.x, new_pose.y)){
        //  let's see if we have already arrived at this position during the search
        auto it = history_map.find(new_pose);
        if (it == history_map.end()){
          history_map[new_pose] = std::make_pair(new_pose_cost, node.getPosition());
          queue.push(Node(&node, new_pose, new_pose_cost));
        }
        else{
          // this is a more efficient path to get to this Pose in the map
          if (new_pose_cost < it->second.first){
            history_map[new_pose].first = new_pose_cost;
            history_map[new_pose].second = node.getPosition();
            queue.push(Node(&node, new_pose, new_pose_cost));
          }
        }
      }
    }
  }
  return SearchResult::NOT_POSSIBLE;
}