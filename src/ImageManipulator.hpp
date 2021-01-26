#pragma once

/**
 * This class helps with all image related operations, such as reading/storing, writing, and displaying images
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common.hpp"

/**
 *  This class helps with all image related operations,
 *  such as reading/storing, writing, and displaying images
 */
class ImageManipulator {
private:
  // input image read from file
  cv::Mat input_mat_;

public:
  ImageManipulator(){};
  ~ImageManipulator(){};

  const cv::Mat& getImage(){
    return input_mat_;
  }

  /**
   * read an image from the given filename
   */
  bool readImage(const std::string& filename) {
    // read image
    input_mat_ = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (input_mat_.empty()) {
      std::cerr << "Failed to read file: " << filename << std::endl;
      return false;
    }
    std::cout << "Image dimensions: " << input_mat_.cols << ", " << input_mat_.rows << std::endl;
    return true;
  }

  /**
   * draw a path on an image
   */
  bool drawPath(cv::Mat &out,
                const Path &path) {
    cv::cvtColor(input_mat_, out, cv::COLOR_GRAY2BGR);

    if (path.size() < 2){
      // std::cerr << "Cannot draw a solution with less than two points" << std::endl;
      return false;
    }

    // display start point
    auto start_it = path.begin();
    out.at<cv::Vec3b>(cv::Point(start_it->x, start_it->y)) = cv::Vec3b(255, 0, 0);
    ++start_it;

    // display end point
    auto end_it = path.end();
    --end_it;
    out.at<cv::Vec3b>(cv::Point(end_it->x, end_it->y)) = cv::Vec3b(0, 255, 0);

    // display path from start to end point
    for (auto it = start_it; it != end_it; ++it) {
      out.at<cv::Vec3b>(cv::Point(it->x, it->y)) = cv::Vec3b(0, 0, 255);
    }
  }

  /**
   * check if the x/y coordinates are inside the image boundaries
   */
  bool checkBounds(int x, int y){
    if (x >= input_mat_.cols || x < 0)
      return false;
    if (y >= input_mat_.rows || y < 0)
      return false;
    return true;
  }

  /**
   * check if the pose is inside the image boundaries
   */
  bool checkBounds(const Pose &pose){
    return checkBounds(pose.x, pose.y);
  }
};
