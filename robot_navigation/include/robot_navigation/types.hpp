/* -------------------------------------------------------------
   Common Data Types for Robot Navigation
   -------------------------------------------------------------
   This header file defines basic data structures used across
   the navigation modules.

   These types are shared by:

       Path Smoother
       Trajectory Generator
       Controller

   Using a separate types file improves modularity and ensures
   consistent data representation across the system.

   Navigation Pipeline Data Flow:

        Waypoints (Point)
             ↓
        Path Smoother
             ↓
        Smoothed Path (Point)
             ↓
        Trajectory Generator
             ↓
        Trajectory Points (TrajectoryPoint)
             ↓
        Controller
--------------------------------------------------------------*/

#ifndef ROBOT_NAVIGATION_TYPES_HPP
#define ROBOT_NAVIGATION_TYPES_HPP


/* -------------------------------------------------------------
   Point Structure
   -------------------------------------------------------------
   Represents a 2D position in the robot's environment.

   Used for:
       - Waypoints
       - Smoothed path points
       - Intermediate path representations

   Coordinates are expressed in the global reference frame
   (typically the "odom" frame in this project).
--------------------------------------------------------------*/

struct Point
{
    double x;   // x-coordinate of the point
    double y;   // y-coordinate of the point
};


/* -------------------------------------------------------------
   TrajectoryPoint Structure
   -------------------------------------------------------------
   Represents a time-parameterized point in the trajectory.

   Unlike the basic Point structure, this also includes
   time information, allowing the robot controller to
   determine when the robot should reach this position.

   Fields:
       x -> x-coordinate
       y -> y-coordinate
       t -> timestamp associated with this point
--------------------------------------------------------------*/

struct TrajectoryPoint
{
    double x;   // x-coordinate of the trajectory point
    double y;   // y-coordinate of the trajectory point
    double t;   // time at which the robot should reach this point
};


#endif
