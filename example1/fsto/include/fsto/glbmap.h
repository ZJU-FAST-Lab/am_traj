#ifndef GLBMAP_H
#define GLBMAP_H

#include "fsto/config.h"

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

class EuclidDistField
{
public:
    EuclidDistField(Eigen::Vector3i xyz, Eigen::Vector3d offset, double scale);
    ~EuclidDistField();
    EuclidDistField(const EuclidDistField &) = delete;

    void setOccupied(const Eigen::Vector3d &pos);
    void updateCubicEDF(void);
    double queryDistSqr(const Eigen::Vector3d &pos) const;
    double queryDist(const Eigen::Vector3d &pos) const;

private:
    Eigen::Vector3i sizeXYZ;
    Eigen::Vector3d originVec;
    double linearScale;

    double *sqrDistsPtr;

    size_t stepX, stepY, stepZ;
    double linearScaleSqr;

    void updateLinearEDF(double *p, size_t step, size_t N, double sqrLinearScale) const;
};

class GlobalMap
{
public:
    GlobalMap(const Config &conf);
    ~GlobalMap();
    void initialize(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool safeQuery(const Eigen::Vector3d &p, double safeRadius) const;

private:
    Config config;
    EuclidDistField *edfPtr;
};

#endif
