#include "fsto/r3planner.h"

using namespace std;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

R3Planner::R3Planner(const Config &conf, std::shared_ptr<const GlobalMap> mapPtr)
    : config(conf), glbMapPtr(mapPtr)
{
}

double R3Planner::plan(const Vector3d &s, const Vector3d &g, vector<Vector3d> &p, double timeout) const
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; i++)
    {
        bounds.setLow(i, std::max(config.r3Bound[i * 2 + 0], std::min(s(i), g(i)) - config.r3SafeRadius * 7.0));
        bounds.setHigh(i, std::min(config.r3Bound[i * 2 + 1], std::max(s(i), g(i)) + config.r3SafeRadius * 7.0));
    }
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));

    si->setStateValidityChecker(
        [&](const ob::State *state) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
            Vector3d position((*pos)[0], (*pos)[1], (*pos)[2]);
            return this->glbMapPtr->safeQuery(position, config.r3SafeRadius);
        });
    si->setup();

    ob::ScopedState<> start(space), goal(space);
    start[0] = s(0);
    start[1] = s(1);
    start[2] = s(2);
    goal[0] = g(0);
    goal[1] = g(1);
    goal[2] = g(2);

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto planner(std::make_shared<og::InformedRRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved;
    solved = planner->ob::Planner::solve(timeout);

    double cost = INFINITY;
    if (solved)
    {
        p.clear();
        const og::PathGeometric path_ = og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
        for (size_t i = 0; i < path_.getStateCount(); i++)
        {
            auto state = path_.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
            p.emplace_back(state[0], state[1], state[2]);
        }
        cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
    }

    return cost;
}

double R3Planner::planOnce(const Vector3d &s, const Vector3d &g, vector<Vector3d> &p) const
{
    return plan(s, g, p, config.searchDuration);
}
