/**
 * Perform an A* search algorithm given an input grayscale image and a start and end point
 *  Uses a simplistic holonomic vehicle model, meaning the system can go in up/down/left/right
 *  and diagonal direction. The heuristic used will be a simple manhattan distance
 *  multiplied by a scalar to help speed up the search at the potential cost of the optimal
 *  path. The scalar can be customized to be more agressive or less agressive
 *
 *     Scalar value > 1 means more agressive, and value of 1 means you will get the optimal path
 *              (barring any rounding errors)
 */

#pragma once

#include <queue>
#include "common.hpp"
#include <opencv2/core.hpp>
#include <map>
#include "Node.hpp"

/**
 * class that performs 2D AStar search given a grayscale input image and
 *      a start/end pose in that image
 */
class AStar
{
private:
  // input grayscale image. Used for detecting occupied pixels
  cv::Mat image_;

  // map from pose to a pair of <cost, neighbor>
  //   I could have done this with another cv::Mat image, but I am going on the assumption
  //   that this will use less space due to the heuristics chosen. We don't need to generate
  //   space for every possible position until we actually need to access it. This map will grow
  //   as we need more space
  std::map< Pose, std::pair<double, Pose> > history_map;

  // store the start and end poses
  Pose   start_pose_;
  Pose   end_pose_;
  size_t total_added_to_queue_;
  size_t total_popped_from_queue_;
  bool   successful_solve_;
  double scalar_;

  // simulate how I could potentially add in vehicle dynamics by setting the possible
  //   set of poses I could transition. This is probably overkill for what we're doing here
  std::list< Pose > possible_motions_cardinal_;
  std::list< Pose > possible_motions_diagonal_;

public:
  AStar(){}
  ~AStar(){};

  // possible return values
  enum SearchResult {
    OK = 0,         // init ok
    FOUND_PATH = 0, // success
    START_INSIDE_OBSTACLE = 1, // start pose is an obstacle pose
    END_INSIDE_OBSTACLE = 2,   // end pose is an obstacle pose
    NOT_POSSIBLE = 3,          // could not find a solution from start to end (ie obstacle in the way)
    INVALID = 4, // start/end out of bounds
  };

  /**
   * initialize the algorithm with the image, start/end points, and an optional heuristic scalar
   */
  SearchResult init(const cv::Mat& image,
            const Pose &start_pose,
            const Pose &end_pose,
            const double &scalar = 1.0);

  /**
   * initialize the list(s) of possible motions as XY coordinates
   */
  void initializeMotions();

  /**
   * print some fun facts about this search
   */
  inline void printSummary() const{
    std::cout << std::endl << "Summary of getting from " << start_pose_ << " to " << end_pose_<< ":" << std::endl;
    std::cout << "            scalar used: " << scalar_ << std::endl;
    std::cout << "  total pushed to queue: " << total_added_to_queue_ << std::endl;
    std::cout << "total popped from queue: " << total_popped_from_queue_ << std::endl;
    std::cout << "       history map size: " << history_map.size() << std::endl;
    std::cout << "                success: " << successful_solve_ << std::endl;
  }

  /**
   * return true if the input image is within the image bounds
   */
  inline bool inBounds(int x, int y){
    if (x < 0 || x >= image_.cols){
      return false;
    }
    if (y < 0 || y >= image_.rows){
      return false;
    }
    return true;
  }

  /**
   * return true if the input image is occupied at the given X/Y coordinates,
   *        or if the input xy coordinate is out of bounds
   */
  inline bool occupied(int x, int y) {
    if (!inBounds(x, y))
      return true;

    if (image_.at<uchar>(cv::Point(x, y)) != 0)
        return true;

    return false;
  }

  /**
   * return true if the input image is occupied at the given Pose
   */
  inline bool occupied(const Pose &pose) {
      return occupied(pose.x, pose.y);
  }

  // create a path from the given node to the start node
  void nodeToPath(const Node &n, Path &out_path);

  SearchResult solve(Path &out_path);
};
