#pragma once
#include <vector>
#include <cmath>
#include "util/pose.hpp"
#include "util/trajectorygenerator.hpp"
#include "chassis.hpp"

class PurePursuitController {
    private:
        int lookAheadDistance; // in inches
        int currentWaypointIndex = 0;
        Chassis *chassis; // Pointer to the chassis controller for robot movement

    public:
        PurePursuitController(Chassis *chassis, int lookAheadDistance = 12)
        : chassis(chassis), lookAheadDistance(lookAheadDistance) {}

        /**
         * @brief Finds the intersection points between a line segment defined by two poses and a circle centered at the origin with radius r.
         * The function returns a vector of Pose objects representing the intersection points that lie within the bounds of the line segment.
         * If no intersection points exist within the bounds, an empty vector is returned.
         * 
         * @param pt1 The starting pose of the line segment.
         * @param pt2 The ending pose of the line segment.
         * @param r The radius of the circle centered at the origin.
         * 
         * @return A vector of Pose objects representing the intersection points within the bounds of the line segment.
         * 
         * For more information on the mathematics behind this function, see:
         * https://mathworld.wolfram.com/Circle-LineIntersection.html
        */
        std::vector<Pose> getBoundedLineCircleIntersection(Pose pt1, Pose pt2, double r);

        /**
         * @brief Given a list of waypoints, the robot's current pose, a starting index in the waypoint list, and a look-ahead distance,
         * this function calculates and returns the goal pose for the robot to pursue using the Pure Pursuit algorithm.
         * 
         * The function iterates through each segment formed by consecutive waypoints, checking for intersections with a circle
         * centered at the robot's current position with a radius equal to the look-ahead distance. If an intersection is found
         * within the bounds of a segment, it is considered as a potential goal pose. The function selects the intersection point
         * that is closest to the next waypoint in the path as the goal pose.
         * 
         * If no valid intersection points are found along any segments, or if all found intersections are behind the robot,
         * the function defaults to returning the last waypoint in the list as the goal pose.
         * 
         * @param waypoints A vector of Pose objects representing the waypoints in the path.
         * @param robotPose The current Pose of the robot.
         *
         * @return A Pose object representing the goal pose for the robot to pursue.
        */
        Pose getGoalPose(std::vector<Pose> waypoints, Pose robotPose, pros::Controller *controller = nullptr);

        void followPath(Trajectory trajectory, pros::Controller *controller = nullptr);

        /**
         * @brief Sets the look-ahead distance for the Pure Pursuit controller.
         * The look-ahead distance determines the distance the goal pose will be from the robot along the path.
         * Alternatively, it can be thought of as the radius of the circle used to find intersection points with path segments.
         * 
         * @param distance The new look-ahead distance in inches.
        */
        void setLookAheadDistance(int distance) { lookAheadDistance = distance; }

        

};
