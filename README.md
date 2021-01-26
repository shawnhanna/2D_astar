# Basic 2D A* algorithm

The algorithm I’m submitting is my own implementation of the A* search algorithm, a commonly used and efficient method for path planning. It is especially useful and commonly used in a small state space with a holonomic 2D robot, such as this challenge.

I believe this algorithm should find a solution if there is one, and produce a failure output if there is not. I have also added a few nice-to-have features to this code that make it easier to use from a UI standpoint.


## Installation:


### Dependencies


*   Open CV (v3.2) - for image reading, writing, and UI - I am using the version that comes with ROS melodic on Ubuntu 18.04
*   C++-11 compatible compiler, such as g++


### Compilation

        cd $PATH_TO_SRC
        mkdir build; cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..
        make


## Running

There are several ways to run the program

 \
With start/end point as arguments:

        ./simple_path_planner path_to_input.png output.png x1 y1 x2 y2

With no start and end points. Just use the OpenCV window to enter points:

        ./simple_path_planner path_to_input.png output.png

To set start/end goal, first select start point with left click, then right click to select end point and start scan. This can be repeated multiple times


### Settings/UI usage:

There are some configurations that can be set for this program:



1. Scalar to use. There are two ways, in the OpenCV window (see the point below) and reading an environment variable that is called SCALAR. ie export SCALAR=10 will set the scalar value to 10
2. OpenCV window: \
Left click to set start pose \
Right click to set end pose and start the search

    Press the 0-9 key to set the scalar multiplier ^ 2:

*   0 = do not use heuristic search, so this is the same as dijkstra search
*   1 = default = manhattan scalar. This is guaranteed to give the shortest path length, but should be faster than scalar 0
*   2->9 = sets the scalar to N^2 (ie N=2 -> scalar = 4, N=7 -> scalar = 49, etc)


## Output

The program has several exit codes depending on the results:

        enum SearchResult {
          OK = 0,         // init ok
          FOUND_PATH = 0, // success
          START_INSIDE_OBSTACLE = 1, // start pose is an obstacle pose
          END_INSIDE_OBSTACLE = 2,   // end pose is an obstacle pose
          NOT_POSSIBLE = 3,          // could not find a solution from start to end (ie obstacle in the way)
          INVALID = 4, // start/end out of bounds
        };

There are also some useful stats that can be used for debugging which scalars to use depending on the use case. It’s a fun and informative way of showing how the A* heuristic can affect the results, both in terms of accuracy and speed.

An example output is here: \
        Finished! cost = 416.328

        Summary of getting from 255, 387 to 463, 378:

                    scalar used: 1
          total pushed to queue: 26844
        total popped from queue: 94952
               history map size: 66773
                        success: 1

        Search finished with code: 0. Displaying the results
        solve - took 201ms
        Found path using 329 vertices

        press q to exit 


## Future Steps/Dynamic Vehicle Model

One limitation of this program is its assumptions about the vehicle and its dynamics. The assumption is that the system is capable of holonomic motion and the size of the robot is only 1 pixel. This is not the usual case for a mobile robot platform, even in academic settings.

To handle the vehicle size limitation, I would have changed the map to ‘expand’ the obstacle pixels by the vehicle width. That should prevent the robot from finding a path that is too close to the obstacles. There may be a better way to do this by factoring width and height into this map modification to get optimal results, but I didn’t have any new thoughts off the top of my head for how that would work.

To add the ability for this solver to perform non-holonomic motions, we would have to do two things:

1. Increase the state space to include directionality of the system (add a theta component, more if needed)
2. Change the state transitions so we only look for possible motions that can be made (use a function that takes the current state and outputs possible transitions from that state)

While writing the code, I tried to make the system fairly modular. I did this by using my own “Pose” class which could be extended to add in more state variables besides x and y. For instance, I could add in a “theta” value. I also added in a list of motions that the robot can make (possible_motions_cardinal_ and possible_motions_diagonal_).

I could improve upon this design and add in a mapping for the transitions from one state to another. For instance, I could take in a variable theta and if the vehicle is non-holonomic, I could only generate states that are possible from that given theta, i.e. the algorithm would not generate a path to a state that is perpendicular to the current theta.

The rest of the algorithm is fundamentally the same. At some point, the state dimensionality will start to hamper the performance of A*. In my younger days, I wrote a (simple) RRT algorithm in python (using pygame) that is more likely more capable of handling high-dimensional state spaces. Feel free to check it out here: [https://github.com/shawnhanna/RRT](https://github.com/shawnhanna/RRT)

