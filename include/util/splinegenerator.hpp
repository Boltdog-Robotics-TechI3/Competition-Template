#include <vector>
#include "pose.hpp"

std::vector<double> linspace(float start, float end, int num);

std::vector<Pose> generateHermiteSpline(Pose startPose, Pose endPose, int numPoints = 100);

std::vector<Pose> generateTrajectory(Pose waypoints[]);