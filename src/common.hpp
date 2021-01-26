#pragma once

/**
 * Commonly used components of the system are stored here, including constants, common types, 
 */
#include <iostream>
#include <list>
#include <math.h>
#include <chrono>

// simple scope timer for printing total processing time for a function
struct MyScopedTimer
{  
  MyScopedTimer(const std::string &name) :
      time_start_(std::chrono::high_resolution_clock::now()),
      name_(name)
  {
  }
  ~MyScopedTimer(void)
  {
    auto time_end = std::chrono::high_resolution_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start_).count();
    std::cout << name_ << " - took " << milliseconds << "ms" <<std::endl;
  }

private:
  std::chrono::high_resolution_clock::time_point time_start_;
  std::string name_;
};


// define a pose as 2 integers, corresponding to an X and Y position in the input image
///  in theory, this class could be any state space
class Pose
{
public:
// empty constructor. default to 0
  Pose() : x(0), y(0)
  {}
  // construct from two ints
  Pose(int _x, int _y) : x(_x), y(_y)
  {}

  inline friend std::ostream &operator<<( std::ostream &output, const Pose &pose ) { 
    output << pose.x << ", " << pose.y;
    return output;
  }

  // equality operation
  inline bool operator ==(const Pose &b) const{
    return b.x == x && b.y == y;
  }

  // poor man's less than operator
  inline bool operator <(const Pose &b) const{
    return (x + y * 1000000) < (b.x + b.y * 1000000);
  }

  int x;
  int y;
};

// add two poses together
inline Pose operator +(const Pose &a, const Pose &b)
{
  return Pose(a.x + b.x, a.y + b.y);
}

// return the (abs) distance between 2 poses
inline double distance(const Pose &p1, const Pose &p2){
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// more efficient way of calculating cost compared to using the distance function
//   between parent and current position.
//   not as scalable if we want to add in more variables to the position state, such
//   as rotation or vehicle dynamics

// cost to move along the diagonal: sqrt(2)
static constexpr const double diagonal_cost = 1.41421356237;
// cost to move up/down/left/right
static constexpr const double cardinal_cost = 1;

// define a path type, which is just a list of the 2d points
typedef std::list< Pose > Path;

