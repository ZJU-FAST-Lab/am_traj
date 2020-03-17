#ifndef R3PLANNER_H
#define R3PLANNER_H

#include "fsto/config.h"
#include "fsto/glbmap.h"

#include <memory>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

class R3Planner
{
public:
    R3Planner(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr);
    double plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double timeout) const;
    double planOnce(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p) const;

private:
    Config config;
    std::shared_ptr<const GlobalMap> glbMapPtr;
};

#endif