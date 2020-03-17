#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "fsto/config.h"
#include "am_traj/am_traj.hpp"
#include "fsto/glbmap.h"

#include <Eigen/Eigen>

class TrajGen
{
public:
    TrajGen(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr);
    Trajectory generate(std::vector<Eigen::Vector3d> &route,
                        Eigen::Vector3d initialVel,
                        Eigen::Vector3d initialAcc,
                        Eigen::Vector3d finalVel,
                        Eigen::Vector3d finalAcc) const;

private:
    Config config;
    std::shared_ptr<const GlobalMap> glbMapPtr;
    AmTraj trajOpt;

    bool trajSafeCheck(const Trajectory &traj, std::vector<Eigen::Vector3d> &route) const;
    std::vector<Eigen::Vector3d> routeSimplify(const std::vector<Eigen::Vector3d> &route, double resolution) const;
};

#endif
