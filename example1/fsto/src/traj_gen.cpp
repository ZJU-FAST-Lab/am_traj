#include "fsto/traj_gen.h"

#include <ctime>

using namespace std;
using namespace Eigen;

TrajGen::TrajGen(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr)
    : config(conf), glbMapPtr(mapPtr),
      trajOpt(config.weightT, config.weightAcc, config.weightJerk,
              config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon)
{
}

Trajectory TrajGen::generate(vector<Vector3d> &route,
                             Vector3d initialVel,
                             Vector3d initialAcc,
                             Vector3d finalVel,
                             Vector3d finalAcc) const
{
    Trajectory traj;

    route = routeSimplify(route, config.spatialResolution);

    if (route.size() < 2 || (route[0] - route[1]).squaredNorm() < FLT_EPSILON)
    {
        return traj;
    }

    int tries = 0;
    do
    {
        clock_t start = clock();

        tries++;
        if (tries > config.tryOut)
        {
            traj.clear();
            break;
        }

        //traj = trajOpt.genOptimalTrajDC(route, initialVel, initialAcc, finalVel, finalAcc);
        //traj = trajOpt.genOptimalTrajDT(route, initialVel, initialAcc, finalVel, finalAcc);
        traj = trajOpt.genOptimalTrajDTC(route, initialVel, initialAcc, finalVel, finalAcc);

        clock_t stop = clock();
        cout << "Fast Spatial-Tenporal Traj Opt: " << (stop - start) * 1000.0 / CLOCKS_PER_SEC << " ms" << endl;
        cout << "Number of Traj Pieces: " << traj.getPieceNum() << endl;

    } while (!trajSafeCheck(traj, route));

    return traj;
}

bool TrajGen::trajSafeCheck(const Trajectory &traj, std::vector<Eigen::Vector3d> &route) const
{
    bool safe = true;

    vector<Vector3d> discretePoints;

    double t;
    int step;
    Vector3d tempTranslation;
    bool tempSafe;
    for (int i = 0; i < traj.getPieceNum(); i++)
    {
        step = traj[i].getDuration() / config.temporalResolution;
        t = 0.0;
        discretePoints.push_back(route[i]);
        for (int j = 0; j < step - 1; j++)
        {
            t += config.temporalResolution;
            tempTranslation = traj[i].getPos(t);
            tempSafe = glbMapPtr->safeQuery(tempTranslation, config.bodySafeRadius);
            safe &= tempSafe;
            if (!tempSafe)
            {
                discretePoints.push_back((route[i] + route[i + 1]) / 2.0);
                break;
            }
        }
    }
    discretePoints.push_back(route[traj.getPieceNum()]);
    route = discretePoints;

    return safe;
}

std::vector<Eigen::Vector3d> TrajGen::routeSimplify(const vector<Vector3d> &route, double resolution) const
{
    vector<Vector3d> subRoute;
    if (route.size() == 1 || route.size() == 2)
    {
        subRoute = route;
    }
    else if (route.size() >= 3)
    {
        vector<Vector3d>::const_iterator maxIt;
        double maxDist = -INFINITY, tempDist;
        Vector3d vec((route.back() - route.front()).normalized());

        for (auto it = route.begin() + 1; it != (route.end() - 1); it++)
        {
            tempDist = (*it - route.front() - vec.dot(*it - route.front()) * vec).norm();
            if (maxDist < tempDist)
            {
                maxDist = tempDist;
                maxIt = it;
            }
        }

        if (maxDist > resolution)
        {
            subRoute.insert(subRoute.end(), route.begin(), maxIt + 1);
            subRoute = routeSimplify(subRoute, resolution);
            vector<Vector3d> tempRoute(maxIt, route.end());
            tempRoute = routeSimplify(tempRoute, resolution);
            subRoute.insert(subRoute.end(), tempRoute.begin() + 1, tempRoute.end());
        }
        else
        {
            subRoute.push_back(route.front());
            subRoute.push_back(route.back());
        }
    }

    return subRoute;
}
