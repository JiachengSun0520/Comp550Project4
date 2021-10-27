///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jiacheng Sun
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <cmath>
namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        // 2 dimension
        return 2;
    }

    //cite from http://ompl.kavrakilab.org/projections.html
    void defaultCellSizes(void) {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.25;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        const double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        projection(0) = values[0];
        projection(1) = values[1];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double theta = q[0];
    const double omega = q[1];
    const double g = 9.81;
    const double* u = control->as<ompl::control::RealVectorControlSpace::ControlType>()-> values;
    const double torque = u[0];

    qdot.resize(q.size(), 0);
    qdot[0] = omega;
    qdot[1] = -g * cos(theta) + torque;
}



// cite from project1 RigidBodyPlanningWithControls.cpp
bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

// https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
// This is a callback method invoked after numerical integration.
void PendulumPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
 {
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
 }


//https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
class DemoControlSpace : public oc::RealVectorControlSpace
 {
 public:
  
     DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
     {
     }
 };

//modified from https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    // R * S
    auto so2(std::make_shared<ompl::base::SO2StateSpace>());
    auto r1(std::make_shared<ompl::base::RealVectorStateSpace>(1));

    ompl::base::RealVectorBounds bounds(1);
    bounds.setLow(-5);
    bounds.setHigh(5);

    r1->setBounds(bounds);
    auto space = so2 + r1;

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    // define a simple setup class
    // oc::SimpleSetup ss(cspace);
    auto ss(std::make_shared<oc::SimpleSetup>(cspace));

    // set state validity checking for this space
     oc::SpaceInformation *si = ss->getSpaceInformation().get();
     ss->setStateValidityChecker(
         [si](const ob::State *state) { return isStateValid(si, state); });



    // set the state propagation routine
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PendulumPostIntegration));

    // set state validity checking for this space
    // ss.setStateValidityChecker(
    //     [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });
    
    // create a start state
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    // ob::ScopedState<ob::SO2StateSpace> start(space);
    start[0] = -M_PI/2;
    start[1] = 0;

    // create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    // ob::ScopedState<ob::SO2StateSpace> goal(space);
    goal[0] = M_PI/2;
    goal[1] = 0;
    
    // set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);
    
    ss->setup();
    
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr & ss , int  choice )
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.

    oc::SimpleSetup *s = ss.get();
    switch(choice) {
        case 1:
            s->setPlanner(std::make_shared<oc::RRT>(s->getSpaceInformation()));
            break;
        case 2:
            auto planner = std::make_shared<oc::KPIECE1>(s->getSpaceInformation());
            s->getStateSpace()->registerProjection("PendulumProjection", ob::ProjectionEvaluatorPtr(new PendulumProjection(s->getStateSpace().get())));
            planner->setProjectionEvaluator("PendulumProjection");
            s->setPlanner(planner);
    }

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        // ss->getSolutionPath().printAsMatrix(std::cout);
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);

    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /*argc*/, char ** /* argv*/)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
