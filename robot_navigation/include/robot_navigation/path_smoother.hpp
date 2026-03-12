/* -------------------------------------------------------------
   Path Smoother Header
   -------------------------------------------------------------
   This header defines the PathSmoother class used in the
   navigation pipeline.

   The purpose of this class is to convert a set of discrete
   waypoints into a smooth continuous path that can be followed
   more easily by the robot.

   Why smoothing is needed:
   Raw waypoints often produce sharp turns which are difficult
   for real robots to follow. Smoothing removes these sharp
   discontinuities and creates a gradual curve.

   This class is used by the controller pipeline:

        Waypoints
            ↓
        PathSmoother
            ↓
        Smoothed Path
            ↓
        Trajectory Generator
            ↓
        Controller
--------------------------------------------------------------*/

#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

/* Standard container for storing path points */
#include <vector>

/* Custom type definitions (Point structure) */
#include "robot_navigation/types.hpp"

/* -------------------------------------------------------------
   PathSmoother Class

   This class provides functionality to smooth a given path.

   It takes a sequence of waypoints and produces a new path
   where intermediate points are adjusted to reduce sharp
   changes in direction.
--------------------------------------------------------------*/
class PathSmoother
{
public:

    /* ---------------------------------------------------------
       smoothPath()

       Input:
           waypoints -> vector of discrete path points

       Output:
           vector of smoothed path points

       Description:
           Applies interpolation and gradient-based smoothing
           to produce a smooth trajectory suitable for robot
           navigation.
    ----------------------------------------------------------*/
    std::vector<Point> smoothPath(const std::vector<Point>& waypoints);

};

#endif
