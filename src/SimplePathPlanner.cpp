
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "common.hpp"
#include "ImageManipulator.hpp"
#include "AStar.hpp"

Pose start_pose;
Pose end_pose;

// handle the mouse clicks in the opencv window
void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    start_pose.x = x;
    start_pose.y = y;
  }
  else if  ( event == cv::EVENT_RBUTTONDOWN )
  {
    std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    end_pose.x = x;
    end_pose.y = y;
    cv::destroyWindow("image");
  }
}

int main(int argc, char const *argv[])
{
  char *scalar_char = getenv("SCALAR");
  double scalar = 1;
  if (scalar_char != NULL){
    scalar = std::stod(std::string(scalar_char));
    std::cout << "Using heuristic scalar: " << scalar << std::endl;
  }

  std::string img_filename, img_output;

  // read args
  if (argc == 3){
    img_filename = std::string(argv[1]);
    img_output = std::string(argv[2]);
  }
  else if (argc == 7){
    img_filename = std::string(argv[1]);
    img_output = std::string(argv[2]);

    start_pose = Pose(std::stoi(argv[3]), std::stoi(argv[4]));
    end_pose = Pose(std::stoi(argv[5]), std::stoi(argv[6]));

    std::cout << "Given start pose: " << start_pose << std::endl;
    std::cout << "Given end pose: " << end_pose << std::endl;
  }
  else {
    std::cerr << "Invalid args. Need: ./simple_path_planner input_filename output_filename [x1 y1 x2 y2]" << std::endl;
    return 100;
  }

  std::cout << "Reading image: " << img_filename << std::endl;
  ImageManipulator manipulator;
  if (!manipulator.readImage(img_filename)){
    std::cerr << "Failed to read image. Exiting" << std::endl;
    return 1;
  }

  if (argc > 2){
    // case where user supplied args
    if (!manipulator.checkBounds(start_pose)){
      std::cerr << "Start pose out of bounds: " << start_pose << std::endl;
      return AStar::INVALID;
    }
    // case where user supplied args
    if (!manipulator.checkBounds(end_pose)){
      std::cerr << "End pose out of bounds: " << end_pose << std::endl;
      return AStar::INVALID;
    }
  }

  AStar::SearchResult search_return;

  // insert end iterator
  int cv_key = 0;
  do {
    std::cout << std::endl << std::endl << std::endl;
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("image", callBackFunc, NULL);

    Path path;
    AStar search_algorithm;
    cv::Mat disp_mat;

    // Don't run if start == end.
    if (start_pose == end_pose){
      search_return = AStar::OK;
    }
    else{
      // initialize the search algorithm with an image, start/end pose, and a scalar for the heuristic
      search_return = search_algorithm.init(manipulator.getImage(), start_pose, end_pose, scalar);
      // check if the input poses were good
      if (search_return == AStar::OK){
        MyScopedTimer timer("solve");
        search_return = search_algorithm.solve(path);
        search_algorithm.printSummary();
        std::cout << "Search finished with code: " << search_return << ". Displaying the results" << std::endl;
      }
      else{
        std::cerr << "Failed to initialize search algorithm" << std::endl;
      }

      // print results
      switch (search_return){
        case AStar::FOUND_PATH:
          if(path.size())
            std::cout << "Found path using " << (path.size() - 2) << " vertices" << std::endl;
          break;
        case AStar::START_INSIDE_OBSTACLE:
          std::cerr << "Could not solve. Start point inside an obstacle" << std::endl;
          break;
        case AStar::END_INSIDE_OBSTACLE:
          std::cerr << "Could not solve. End point is inside an obstacle" << std::endl;
          break;
        case AStar::NOT_POSSIBLE:
          std::cerr << "Could not solve. There is no solution" << std::endl;
          break;
      }
    }

    // draw the path to the output image
    manipulator.drawPath(disp_mat, path);
    std::cout << "Writing image: " << img_output << std::endl;
    cv::imwrite(img_output, disp_mat);

    // opencv UI used
    if (argc == 3){
      // show image and wait for user input to continue
      cv::imshow("image", disp_mat);
      std::cout << "press q to exit " << std::endl;
      cv_key = cv::waitKey(-1);

      // A new scalar was requested
      if (cv_key >= (int)'0' && cv_key <= (int)'9'){
        int key = (double)(cv_key - (int)'0');
        // scalar = std::max(1, key * key);
        scalar = key * key;
        std::cout << "Setting scalar : " << scalar << std::endl;
      }
      // image output
    }
    else
    {
      // just one-time run w/ command line args
      break;
    }
    
  } while(cv_key != (int)'q');

  return (int)search_return;
}
