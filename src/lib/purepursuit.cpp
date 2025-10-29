#include <vector>
#include <cmath>
#include <algorithm>
#include "purepursuit.hpp"

void PurePursuitController::reset() {
    this->currentWaypointIndex = 0;
}

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
std::vector<Pose> PurePursuitController::getBoundedLineCircleIntersection(Pose pt1, Pose pt2, double r) {
    // Set up Pose Array
    std::vector<Pose> intersections;
    
    // Math Stuff
    double dx = pt2.getX() - pt1.getX();
    double dy = pt2.getY() - pt1.getY();
    double dr = sqrt(dx*dx + dy*dy);
    double D = pt1.getX()*pt2.getY() - pt2.getX()*pt1.getY();
    double discriminant = (r*r * dr*dr) - (D*D);

    // If the discriminant is at least 0, intersections exist. Otherwise, just return an empty Pose Array.
    if (discriminant >= 0) {

        // Calculate the x and y coordinates of the two intersection points.
        double posX = (D * dy + std::copysign(1, dy) * dx * sqrt(discriminant)) / (dr*dr);
        double posY = (-D * dx + std::abs(dy) * sqrt(discriminant)) / (dr*dr);

        double negX = (D * dy - std::copysign(1, dy) * dx * sqrt(discriminant)) / (dr*dr);
        double negY = (-D * dx - std::abs(dy) * sqrt(discriminant)) / (dr*dr);

        // Find the maxes and mins for the two poses that define the line. This will be used for bounding.
        double maxX = std::max(pt1.getX(), pt2.getX());
        double maxY = std::max(pt1.getY(), pt2.getY());
        double minX = std::min(pt1.getX(), pt2.getX());
        double minY = std::min(pt1.getY(), pt2.getY());

        // if the intersection point is between pt1 and pt2 on the line, it is valid.
        if ((posX >= minX && posX <= maxX) && (posY >= minY && posY <= maxY)) {
            intersections.push_back(Pose(posX, posY, 0));
        }

        if ((negX >= minX && negX <= maxX) && (negY >= minY && negY <= maxY)) {
            intersections.push_back(Pose(negX, negY, 0));
        }        
    }

    return intersections;
}

Pose PurePursuitController::getGoalPose(std::vector<Pose> waypoints, Pose robotPose) {
    Pose goalPose;
    goalPose.setPose(0, 0, 0); // Default to an invalid pose
    Pose temp;
    std::vector<Pose> intersections;

    Pose offsetWaypoint1;
    Pose offsetWaypoint2;

    for (int i = currentWaypointIndex; i < waypoints.size() - 1; i++) {
        // pros::delay(500);

        // Offset the waypoints by the robot's current position. This effectively places the robot at the origin.
        offsetWaypoint1 = waypoints[i].negativeOffset(robotPose);
        offsetWaypoint2 = waypoints[i+1].negativeOffset(robotPose);

        // Check to see if the line between the current two waypoints intersects our look ahead circle.
        intersections = getBoundedLineCircleIntersection(offsetWaypoint1, offsetWaypoint2, lookAheadDistance);

        // If no intersections were found, move on to the next set of waypoints.
        // Also update the current waypoint index to optimize future searches.
        if (intersections.size() == 0) {
            currentWaypointIndex++;
            continue;
        }

        // Translate the intersection points back to the original coordinate system.
        for (int j = 0; j < intersections.size(); j++) {
            intersections[j] = intersections[j].offset(robotPose);
        }

        // If two intersections were found, store whichever one is closer to the next waypoint in the path.
        if (waypoints.size() > 1 && waypoints[i+1].distanceTo(intersections[0]) > waypoints[i+1].distanceTo(intersections[1])) {
            temp = intersections[1];
        }
        else {
            temp = intersections[0];
        }

        // If the next waypoint is closer to the robot then the temp goal pose, discard that goal pose and move to the next set of waypoints.
        // Also update the current waypoint index to optimize future searches.
        if (waypoints[i+1].distanceTo(robotPose) < waypoints[i+1].distanceTo(temp)) {
            currentWaypointIndex++;
            continue;
        }
        else {
            goalPose = temp;
            break;
        }
    }

    // If no valid goal pose was found, just return the last waypoint in the path.
    if (goalPose.getX() == 0 && goalPose.getY() == 0) {
        goalPose = waypoints[waypoints.size() - 1];
    }

    return goalPose;
}

void PurePursuitController::followPath(Trajectory trajectory, pros::Controller *controller) {
    std::vector<Pose> waypoints = trajectory.getSplineWaypoints();
    Pose robotPose = chassis->getPose(); // Get the robot's current pose from the chassis
    Pose goalPose;

    double linearError;
    double angularError;
    double absTargetAngle;
    double leftOutput;
    double rightOutput;

    while (robotPose.distanceTo(waypoints[waypoints.size() - 1]) > 1) { // Continue until the robot is within 2 inches of the last waypoint
        goalPose = this->getGoalPose(waypoints, robotPose);

        controller->print(0, 0, "X: %.3f Y: %.3f ", goalPose.getX(), goalPose.getY());

        linearError = robotPose.distanceTo(goalPose);

        absTargetAngle = robotPose.angleTo(goalPose);
		absTargetAngle = absTargetAngle < 0 ? absTargetAngle + M_TWOPI : absTargetAngle;
		
		angularError = absTargetAngle - chassis->getWorldFrameHeading();
		if (angularError > M_PI || angularError < (-1 * M_PI)) {
			angularError = -1 * std::copysign(1, angularError) * (M_TWOPI - abs(angularError));
		}
		
        leftOutput = std::clamp(linearError * 6, -40.0, 40.0) - angularError * 15;
        rightOutput = std::clamp(linearError * 6, -40.0, 40.0) + angularError * 15;

        // leftOutput = std::clamp(leftOutput, -70.0, 70.0);
        // rightOutput = std::clamp(rightOutput, -70.0, 70.0);

        // controller->print(0, 0, "L %.2f R %.2f", leftOutput, rightOutput);

        chassis->tank(leftOutput, rightOutput);
        
        robotPose = chassis->getPose(); // Update the robot's current pose

        pros::delay(20);
    }
}

