/* -------------------------------------------------------------
   Path Smoother
   -------------------------------------------------------------
   This module converts a set of discrete waypoints into a
   smooth continuous path that can be followed by the robot.

   The smoothing process is performed in two stages:

   1) Path Interpolation
      - Inserts intermediate points between waypoints
      - Ensures the path has uniform spatial resolution

   2) Gradient-based Smoothing (Laplacian smoothing)
      - Adjusts intermediate points iteratively
      - Removes sharp corners
      - Produces a smooth curve while preserving path shape

   This technique is commonly used in mobile robot navigation
   to improve trajectory tracking stability.
--------------------------------------------------------------*/

#include "robot_navigation/path_smoother.hpp"
#include <cmath>

/* -------------------------------------------------------------
   smoothPath()

   Input:
       waypoints -> discrete path points

   Output:
       smooth path with higher resolution and smoother geometry
--------------------------------------------------------------*/

std::vector<Point> PathSmoother::smoothPath(const std::vector<Point>& waypoints)
{
    /* ---------------------------------------------------------
       Dense path after interpolation
       ---------------------------------------------------------
       This vector will store the interpolated points between
       the original waypoints.
    ----------------------------------------------------------*/
    std::vector<Point> dense;

    /* ---------------------------------------------------------
       Step 1: Interpolate intermediate points
       ---------------------------------------------------------
       The goal is to increase the spatial resolution of the
       path so that smoothing works effectively.

       resolution = distance between consecutive interpolated
                    points along the path.
    ----------------------------------------------------------*/

    double resolution = 0.05;

    for(size_t i = 0; i < waypoints.size()-1; i++)
    {
        /* Current waypoint and next waypoint */
        Point p1 = waypoints[i];
        Point p2 = waypoints[i+1];

        /* Vector between the two points */
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        /* Euclidean distance between points */
        double dist = std::sqrt(dx*dx + dy*dy);

        /* Number of interpolation steps required */
        int steps = std::max(1, int(dist / resolution));

        /* Generate intermediate points */
        for(int j = 0; j < steps; j++)
        {
            /* Normalized interpolation factor */
            double t = (double)j / steps;

            Point p;

            /* Linear interpolation between p1 and p2 */
            p.x = p1.x + t * dx;
            p.y = p1.y + t * dy;

            dense.push_back(p);
        }
    }

    /* Add the final waypoint explicitly */
    dense.push_back(waypoints.back());


    /* ---------------------------------------------------------
       Step 2: Gradient-based smoothing
       ---------------------------------------------------------
       This algorithm iteratively adjusts points to remove
       sharp angles while keeping them close to the original
       path.

       smooth = working copy of the path
    ----------------------------------------------------------*/

    std::vector<Point> smooth = dense;

    /* Weight controlling how strongly points stay near original */
    double weight_data = 0.1;

    /* Weight controlling smoothing influence from neighbors */
    double weight_smooth = 0.3;

    /* Convergence threshold */
    double tolerance = 0.00001;

    double change = tolerance;

    /* ---------------------------------------------------------
       Iteratively update points until convergence
    ----------------------------------------------------------*/
    while(change >= tolerance)
    {
        change = 0.0;

        /* Skip first and last point to preserve endpoints */
        for(size_t i = 1; i < smooth.size()-1; i++)
        {
            double x_old = smooth[i].x;
            double y_old = smooth[i].y;

            /* ----------------------------------------------
               Update X coordinate

               Term 1: pull point towards original dense path
               Term 2: average with neighbors (smoothness)
            -----------------------------------------------*/

            smooth[i].x +=
                weight_data * (dense[i].x - smooth[i].x) +
                weight_smooth * (smooth[i-1].x + smooth[i+1].x - 2*smooth[i].x);

            /* Same smoothing update applied to Y coordinate */

            smooth[i].y +=
                weight_data * (dense[i].y - smooth[i].y) +
                weight_smooth * (smooth[i-1].y + smooth[i+1].y - 2*smooth[i].y);

            /* Track how much the path changed */
            change += std::fabs(x_old - smooth[i].x);
            change += std::fabs(y_old - smooth[i].y);
        }
    }

    /* Return the smoothed path */
    return smooth;
}
