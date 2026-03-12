/* -------------------------------------------------------------
   Trajectory Generator Header
   -------------------------------------------------------------
   This header defines the TrajectoryGenerator class used in
   the navigation pipeline.

   Purpose:
   A robot cannot follow only a geometric path (x,y). Instead,
   it must follow a trajectory that specifies where the robot
   should be at a particular time.

   The trajectory generator converts a smoothed path into a
   time-parameterized trajectory.

   Navigation Pipeline:

        Waypoints
            ↓
        Path Smoother
            ↓
        Smoothed Path
            ↓
        Trajectory Generator
            ↓
        Time-parameterized trajectory (x, y, t)
            ↓
        Controller (PID tracking)

--------------------------------------------------------------*/

#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

/* Standard vector container for storing trajectory points */
#include <vector>

/* Custom types used in the navigation system */
#include "robot_navigation/types.hpp"

/* -------------------------------------------------------------
   TrajectoryGenerator Class

   This class generates a trajectory from a given path by
   assigning timestamps to each path point based on a desired
   constant velocity.

   The resulting trajectory consists of points with position
   and time information, allowing the controller to follow the
   motion in a time-consistent manner.
--------------------------------------------------------------*/

class TrajectoryGenerator
{

public:

    /* ---------------------------------------------------------
       generateTrajectory()

       Input:
           path     -> smoothed geometric path (x, y)
           velocity -> desired robot velocity (m/s)

       Output:
           vector of trajectory points containing:
               x : x-coordinate
               y : y-coordinate
               t : timestamp

       Description:
           The function computes the time required to travel
           between consecutive path points using:

                time = distance / velocity

           The cumulative time is then assigned to each point
           to create a time-parameterized trajectory.
    ----------------------------------------------------------*/

    std::vector<TrajectoryPoint> generateTrajectory(
        const std::vector<Point>& path,
        double velocity);

};

#endif
