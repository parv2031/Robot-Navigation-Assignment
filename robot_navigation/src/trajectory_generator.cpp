/* -------------------------------------------------------------
   Trajectory Generator
   -------------------------------------------------------------
   This module converts a geometric path into a time-parameterized
   trajectory.

   Input:
       - A smoothed path consisting of spatial points (x, y)
       - A desired constant velocity for the robot

   Output:
       - A trajectory consisting of points (x, y, t)

   Purpose:
       Robots do not follow just spatial paths; they must follow
       trajectories that specify where the robot should be at a
       particular time.

   The algorithm assigns timestamps to path points based on
   the distance between consecutive points and the desired
   velocity.
--------------------------------------------------------------*/

#include "robot_navigation/trajectory_generator.hpp"
#include <cmath>

/* -------------------------------------------------------------
   generateTrajectory()

   Parameters:
       path     -> smoothed path (vector of 2D points)
       velocity -> constant robot velocity (m/s)

   Returns:
       traj     -> vector of trajectory points (x, y, time)
--------------------------------------------------------------*/

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
        const std::vector<Point>& path,
        double velocity)
{

    /* ---------------------------------------------------------
       Vector to store generated trajectory points
    ----------------------------------------------------------*/
    std::vector<TrajectoryPoint> traj;

    /* Time accumulator for trajectory */
    double time = 0.0;

    /* ---------------------------------------------------------
       Iterate through path segments
       ---------------------------------------------------------
       Each pair of points forms a segment. The time required
       to travel that segment depends on its length and the
       chosen velocity.
    ----------------------------------------------------------*/

    for(size_t i=0;i<path.size()-1;i++)
    {

        /* Difference between consecutive points */
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;

        /* Euclidean distance between points */
        double dist = std::sqrt(dx*dx + dy*dy);

        /* -----------------------------------------------------
           Compute time required to travel this segment

           dt = distance / velocity
        ------------------------------------------------------*/
        double dt = dist / velocity;

        /* Create trajectory point */
        TrajectoryPoint p;

        /* Position from the current path point */
        p.x = path[i].x;
        p.y = path[i].y;

        /* Timestamp assigned to this point */
        p.t = time;

        /* Add to trajectory */
        traj.push_back(p);

        /* Update cumulative time for next segment */
        time += dt;
    }

    /* Return the generated trajectory */
    return traj;
}
