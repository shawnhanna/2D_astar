#pragma once

#include "common.hpp"


/**
 * a small node class that stores the position, parent node, and cost of getting to this node
 */
class Node
{
private:
  /* data */
public:
  Node( const Node *parent, const Pose &position, double score) :
                parent_(parent),
                position_(position),
                cost_so_far_(score)
  {}

  // copy constructor
  Node( const Node &other) : parent_(other.getParent()),
                             position_(other.getPosition()),
                             cost_so_far_(other.getCost())
  {}

  // root node only!
  Node( const Pose &position ) :
                cost_so_far_(0),
                parent_(0),
                position_(position)
  {}

  ~Node(){};

  // return the cost so far for this node, meaning the number of lines added to its built-up list
  inline double getCost() const {
    return cost_so_far_;
  }

  static void setHeuristicScalar(const double &scalar){
    scalar_ = scalar;
  }
  static void setEndPose(const Pose &end_pose){
    end_pose_ = end_pose;
  }

  // calculate a heuristic cost so we can speed up the order of the search
  // give the end point so we don't store that internally in the node
  inline double getHeuristicCost() const {
    return getCost() + distance(position_, end_pose_) * scalar_;
  }

  Pose getPosition() const {
    return position_;
  }

  const Node *getParent() const {
    return parent_;
  }

private:
  //  Using a static scalar for the heuristic could have issues if we are running multiple
  //  searches at the same time with different scalars, but I'm going for "most readable"
  //  code for this exercise and this is simple, efficient, and effective
  static double scalar_;

  // do the same for the End Pose. I was having trouble getting my comparator function
  //  working without doing it this way, and I dont want to waste time figuring that out
  //  until later if this solution will work well for now.
  static Pose end_pose_;

  double cost_so_far_;

  const Node *parent_;
  Pose position_;
};
