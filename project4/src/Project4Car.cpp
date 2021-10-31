///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Yinjie Xu, Jiacheng Sun
//////////////////////////////////////

#include <iostream>
#include <cmath>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <fstream>

#include <ompl/tools/benchmark/Benchmark.h>

#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"
namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the car
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ob::State *  state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        // auto double* values = state->as<ob::CompoundStateSpace::StateType>() -> values;
        auto compoundState = state->as<ob::CompoundStateSpace::StateType>();
        auto se2 = compoundState->as<ob::SE2StateSpace::StateType>(0);
        auto r2Value = se2->as<ob::RealVectorStateSpace::StateType>(0) ->values;
        projection(0) = r2Value[0];
        projection(1) = r2Value[1];

    }
};

void carODE(const oc::ODESolver::StateType & q, const oc::Control * control,
            oc::ODESolver::StateType & qdot)
{
    const double theta = q[2];
    const double v = q[3];

    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double omega = u[0];
    const double vdot = u[1];


    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);

    qdot[0] = v * cos(theta);            // xdot
    qdot[1] = v * sin(theta);            // ydot
    qdot[2] = omega;
    qdot[3] = vdot;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.

    Rectangle rect1;
    rect1.x = -10;
    rect1.y = -4;
    rect1.width = 8.0;
    rect1.height = 8.0;
    obstacles.push_back(rect1);

    Rectangle rect2;
    rect2.x = 1.0;
    rect2.y = -4.0;
    rect2.width = 8.0;
    rect2.height = 8.0;
    obstacles.push_back(rect2);

    Rectangle rect3;
    rect3.x = -10;
    rect3.y = 6.0;
    rect3.width = 20;
    rect3.height = 2.0;
    obstacles.push_back(rect3);

    Rectangle rect4;
    rect4.x = -10;
    rect4.y = -8.0;
    rect4.width = 20;
    rect4.height = 2.0;
    obstacles.push_back(rect4);

    Rectangle rect5;
    rect5.x = -10;
    rect5.y = -8.0;
    rect5.width = 2;
    rect5.height = 15;
    obstacles.push_back(rect5);

    Rectangle rect6;
    rect6.x = 8.5;
    rect6.y = -8.0;
    rect6.width = 2;
    rect6.height = 15.0;
    obstacles.push_back(rect6);
}

// use point to do collision checking as pdf said
bool isValidStateCar(oc::SpaceInformation* si, const ob::State *state, const std::vector<Rectangle> &obstacles)
{
    // check for collisions in the state space
    auto compoundState = state->as<ob::CompoundStateSpace::StateType>();
    auto se2 = compoundState->as<ob::SE2StateSpace::StateType>(0);
    auto r2 = se2->as<ob::RealVectorStateSpace::StateType>(0);


    return isValidStatePoint(r2, obstacles) && si->satisfiesBounds(state);
    // return isValidStateSquare(se2, 0.1, obstacles) && si->satisfiesBounds(state);
}

// https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
// This is a callback method invoked after numerical integration.
void carPostIntegration(const ob::State* /*state*/, const oc::Control * /*control*/, const double /*duration*/, ob::State *result)
{
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1));
}

oc::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds se2bounds(2);
    se2bounds.setLow(-10);
    se2bounds.setHigh(10);
    se2->setBounds(se2bounds);

    // velocity
    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds r1bounds(1);
    r1bounds.setLow(0);
    r1bounds.setHigh(2);
    r1->setBounds(r1bounds);

    //configuration space and velocity space
    auto space = se2 + r1;

    // obstacles
    makeStreet(obstacles);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    ob::RealVectorBounds cbounds(2);
    // omega
    cbounds.setLow(0, -2);
    cbounds.setHigh(0, 2);
    // v
    cbounds.setLow(1, 0);
    cbounds.setHigh(1, 1);
    cspace->setBounds(cbounds);

    // define a simple setup class
    // oc::SimpleSetup ss(cspace);
    //oc::SimpleSetupPtr ss(new oc::SimpleSetup(cspace));
    auto ss(std::make_shared<oc::SimpleSetup>(cspace));

    // // set the state propagation routine
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
    //ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    // set state validity checking for this space
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    //auto si(std::make_shared<ob::SpaceInformation>(space));
    ss->setStateValidityChecker([obstacles, si](const ob::State *state) {
        return isValidStateCar(si, state, obstacles);
    });


    // start state
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = -6;
    start[1] = -5;
    start[2] = 0;
    start[3] = 0;


    // goal state
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal[0] = 6;
    goal[1] = 5;
    goal[2] = 0;
    goal[3] = 0;

    ss->setStartAndGoalStates(start, goal, 0.05);

    return ss;
}

void planCar(oc::SimpleSetupPtr & ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    oc::SimpleSetup *s = ss.get();
    switch (choice){
        case 1:
            s->setPlanner(std::make_shared<oc::RRT>(s->getSpaceInformation()));
            break;
        case 2:
            auto planner = std::make_shared<oc::KPIECE1>(s->getSpaceInformation());
            s->getStateSpace()->registerProjection("CarProjection", ob::ProjectionEvaluatorPtr(new CarProjection(s->getStateSpace().get())));
            planner->setProjectionEvaluator("CarProjection");
            s->setPlanner(planner);
            break;


    }
    
    ss->setup();
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved) {
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

}

void benchmarkCar(oc::SimpleSetupPtr /*& ss*/)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    oc::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}

