#pragma once
#include <vector>
#include <cmath>
#include "util/pose.hpp"

class Trajectory {
    private:
        std::vector<Pose> baseWaypoints;
        std::vector<Pose> splineWaypoints;

    public:
        Trajectory(std::vector<Pose> baseWaypoints, std::vector<Pose> splineWaypoints)
            : baseWaypoints(baseWaypoints), splineWaypoints(splineWaypoints) {}

        std::vector<Pose> getBaseWaypoints() const { return baseWaypoints; }

        std::vector<Pose> getSplineWaypoints() const { return splineWaypoints; }
};