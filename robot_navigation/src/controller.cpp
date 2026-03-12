/* -------------------------------------------------------------
   Controller Node for Trajectory Tracking
   -------------------------------------------------------------
   This ROS2 node implements a trajectory tracking controller
   for a differential drive robot (TurtleBot3).

   The controller performs the following steps:

   1. Reads discrete waypoints from ROS2 parameters
   2. Applies path smoothing to generate a continuous path
   3. Generates a time-parameterized trajectory
   4. Uses a PID controller to track the trajectory
   5. Publishes velocity commands to /cmd_vel
   6. Publishes planned and actual paths for visualization

   Visualization topics:
   /planned_path  -> smoothed path
   /robot_path    -> actual robot trajectory
--------------------------------------------------------------*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <cmath>
#include <algorithm>

/* Custom project headers */
#include "robot_navigation/types.hpp"
#include "robot_navigation/path_smoother.hpp"
#include "robot_navigation/trajectory_generator.hpp"


/* -------------------------------------------------------------
   ControllerNode class
   -------------------------------------------------------------
   This node subscribes to odometry and generates velocity
   commands to follow a trajectory generated from waypoints.
--------------------------------------------------------------*/

class ControllerNode : public rclcpp::Node
{

public:

    /* ---------------------------------------------------------
       Constructor
       ---------------------------------------------------------
       Initializes publishers, subscribers and parameters.
    ----------------------------------------------------------*/
    ControllerNode() : Node("trajectory_controller")
    {

        /* Publisher for the smoothed planned path */
        planned_path_pub =
            create_publisher<nav_msgs::msg::Path>("/planned_path",10);

        /* Publisher for the actual robot trajectory */
        robot_path_pub =
            create_publisher<nav_msgs::msg::Path>("/robot_path",10);

        /* Define reference frame for robot path */
        robot_path.header.frame_id = "odom";

        /* Velocity command publisher */
        cmd_pub =
            create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

        /* Subscribe to odometry to get robot state */
        odom_sub =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&ControllerNode::odomCallback,this,std::placeholders::_1));

        /* -----------------------------------------------------
           Waypoints are provided through ROS parameters
           instead of hardcoding them in the code.
           This allows flexible trajectory configuration.
        ------------------------------------------------------*/
        declare_parameter<std::vector<double>>("waypoints_x");
        declare_parameter<std::vector<double>>("waypoints_y");
    }

private:

    /* Publishers */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub;

    /* Stores actual robot path */
    nav_msgs::msg::Path robot_path;

    /* Velocity command publisher */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    /* Odometry subscriber */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    /* Generated trajectory (x,y,t) */
    std::vector<TrajectoryPoint> trajectory;

    /* Flag to ensure trajectory is initialized only once */
    bool trajectory_initialized = false;

    /* Index of the current target trajectory point */
    size_t current_index = 0;

    /* ---------------------------------------------------------
       PID Controller Gains
       ---------------------------------------------------------
       Kp_d, Kd_d  -> control forward velocity (distance error)
       Kp_a, Kd_a  -> control angular velocity (heading error)
    ----------------------------------------------------------*/

    double Kp_d = 0.8;
    double Kd_d = 0.1;

    double Kp_a = 1.2;
    double Kd_a = 0.2;

    /* Store previous errors for derivative terms */
    double prev_d = 0;
    double prev_a = 0;

    /* ---------------------------------------------------- */

    /* ---------------------------------------------------------
       initializeTrajectory()
       ---------------------------------------------------------
       Reads waypoint parameters, smooths the path,
       generates trajectory points, and publishes
       the planned path for visualization.
    ----------------------------------------------------------*/

    void initializeTrajectory()
    {

        /* Retrieve waypoint parameters */
        std::vector<double> wp_x;
        std::vector<double> wp_y;

        get_parameter("waypoints_x", wp_x);
        get_parameter("waypoints_y", wp_y);

        /* Convert parameter arrays into Point objects */
        std::vector<Point> waypoints;

        for(size_t i = 0; i < wp_x.size(); i++)
        {
            Point p;
            p.x = wp_x[i];
            p.y = wp_y[i];
            waypoints.push_back(p);
        }

        /* -------------------------------------------------
           Apply path smoothing algorithm
           -------------------------------------------------
           Converts discrete waypoints into a smooth path
        --------------------------------------------------*/
        PathSmoother smoother;

        auto smooth_path = smoother.smoothPath(waypoints);

        /* -------------------------------------------------
           Publish smoothed path for visualization in RViz
        --------------------------------------------------*/
        nav_msgs::msg::Path path_msg;

        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = now();

        for(auto &p : smooth_path)
        {

            geometry_msgs::msg::PoseStamped pose;

            pose.header.frame_id = "odom";
            pose.header.stamp = now();

            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;

            path_msg.poses.push_back(pose);
        }

        planned_path_pub->publish(path_msg);

        /* -------------------------------------------------
           Generate time-parameterized trajectory
           -------------------------------------------------
           Adds timestamps to the smoothed path
        --------------------------------------------------*/
        TrajectoryGenerator generator;

        trajectory =
            generator.generateTrajectory(smooth_path,0.4);
    }

    /* ---------------------------------------------------- */

    /* ---------------------------------------------------------
       odomCallback()
       ---------------------------------------------------------
       Called every time new odometry data is received.

       Steps:
       1. Initialize trajectory if not already done
       2. Check if goal reached
       3. Compute robot orientation
       4. Select target trajectory point
       5. Compute distance and heading errors
       6. Apply PID controller
       7. Publish velocity command
       8. Record robot path for visualization
    ----------------------------------------------------------*/

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        /* Current robot position */
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        /* Initialize trajectory only once */
        if(!trajectory_initialized)
        {
            initializeTrajectory();
            trajectory_initialized = true;
        }

        /* If trajectory is empty, exit */
        if(trajectory.empty())
            return;

        /* -------------------------------------------------
           Goal condition
           Robot stops when close to final trajectory point
        --------------------------------------------------*/

        Point goal;
        goal.x = trajectory.back().x;
        goal.y = trajectory.back().y;

        double goal_dist = std::hypot(goal.x - x, goal.y - y);

        if(goal_dist < 0.05)
        {
            geometry_msgs::msg::Twist stop;
            cmd_pub->publish(stop);
            return;
        }

        /* -------------------------------------------------
           Extract robot orientation (yaw angle)
        --------------------------------------------------*/

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        tf2::Quaternion q(qx,qy,qz,qw);

        tf2::Matrix3x3 m(q);

        double roll,pitch,theta;

        m.getRPY(roll,pitch,theta);

        /* -------------------------------------------------
           Select target trajectory point
        --------------------------------------------------*/

        auto target = trajectory[current_index];

        double dx = target.x - x;
        double dy = target.y - y;

        double dist = std::hypot(dx,dy);

        /* Desired heading towards target */
        double desired_heading = atan2(dy,dx);

        /* Heading error */
        double heading_error = desired_heading - theta;

        /* Normalize heading error to [-pi, pi] */
        while(heading_error > M_PI) heading_error -= 2*M_PI;
        while(heading_error < -M_PI) heading_error += 2*M_PI;

        /* -------------------------------------------------
           Move to next trajectory point when close enough
        --------------------------------------------------*/

        if(dist < 0.15 && current_index < trajectory.size()-1)
            current_index++;

        /* -------------------------------------------------
           PID Control
        --------------------------------------------------*/

        double dd = dist - prev_d;
        double da = heading_error - prev_a;

        double v = Kp_d * dist + Kd_d * dd;
        double w = Kp_a * heading_error + Kd_a * da;

        prev_d = dist;
        prev_a = heading_error;

        /* Limit velocities */
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = std::min(0.4,v);
        cmd.angular.z = std::clamp(w,-1.5,1.5);

        /* -------------------------------------------------
           Record actual robot trajectory for RViz
        --------------------------------------------------*/

        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = "odom";
        pose.header.stamp = now();

        pose.pose.position.x = x;
        pose.pose.position.y = y;

        robot_path.poses.push_back(pose);

        robot_path.header.stamp = now();

        robot_path_pub->publish(robot_path);

        /* Publish velocity command */
        cmd_pub->publish(cmd);
    }
};

/* ---------------------------------------------------- */

/* ---------------------------------------------------------
   Main function
   ---------------------------------------------------------
   Initializes ROS2 and spins the controller node.
----------------------------------------------------------*/

int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}
