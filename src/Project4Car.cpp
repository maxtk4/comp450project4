///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast & Max Kuhlman
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // The dimension of your projection for the car
        return 3;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
		auto compoundState = state->as<ob::CompoundStateSpace::StateType>();

        // Get the SE(2) state and velocity as projections
		auto configurationState = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
		projection(0) = configurationState->getX();
		projection(1) = configurationState->getY();
		projection(2) = configurationState->getYaw();

		// auto velocityState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
		// projection(4) = velocityState->values[0];
    }
};

void carODE(const oc::ODESolver::StateType &q, const oc::Control *control,
            oc::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
	// Useful https://ompl.kavrakilab.org/odeint.html
	// ompl::base::State values are translated into an iterable container of real values
	// q and qdot are vectors, like if you had called ompl::base::ScopedState::reals
	// the ompl::control::Control can be cast as an ompl::control::RealVectorControlSpace::ControlType to expose a vector property called values
	// std::cout << "Inside carODE" << std::endl;
	const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

	
	const double omega = u[0];
	const double vdot = u[1];
	
	// The order of the elements of q and qdot depends on the base Space
	// For the CarSpace, since it's an SE(2) space and then a realvectorspace(1), it's x, y, theta, v
	// const double x = q[0];
	// const double y = q[1];
	const double theta = q[2];
	const double v = q[3];
	
	// Ensure that qdot is the same size as q and set all values to zero
	qdot.resize(q.size(), 0);
	
	// Assign the values of qdot according to our dynamical laws
	qdot[0] = v * cos(theta);
	qdot[1] = v * sin(theta);
	qdot[2] = omega;
	qdot[3] = vdot;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // Fill in the vector of rectangles with the demo environment from the assignment
	Rectangle rect;
	
	// bottom 'building'
    rect.x = -10;
    rect.y = -10;
    rect.height = 4;
    rect.width = 20;
    obstacles.push_back(rect);

    // left 'building'
    rect.x = -10;
    rect.y = -4;
    rect.height = 8;
    rect.width = 9;
    obstacles.push_back(rect);

    // right 'building'
    rect.x = 1;
    rect.y = -4;
    rect.height = 8;
    rect.width = 9;
    obstacles.push_back(rect);

    // top 'building'
    rect.x = -10;
    rect.y = 6;
    rect.height = 4;
    rect.width = 20;
    obstacles.push_back(rect);
}

bool isStateValid(const oc::SpaceInformation* spaceInformation, const std::vector<Rectangle> obstacles, const ompl::base::State* state) {
	// Get the SE(2) state, which contains the x, y coordinates

	// First cast to a compound state s.t. the SE(2) space can be extracted
	auto compoundState = state->as<ob::CompoundStateSpace::StateType>();
	auto SE2State = compoundState->as<ob::SE2StateSpace::StateType>(0);
	// Check bounds and point collision
    return spaceInformation->satisfiesBounds(state) && isValidPoint(SE2State->getX(), SE2State->getY(), obstacles);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles, double vmax = 3.0, double omegamax = 3.1415927, double vdotmax = 2.0)
{
    // Create and setup the car's state space, control space, validity checker, everything you need for planning.

    // Create a state space
	// This state space needs an SE(2) space for the x,y,theta configuration of the robot, and a RealVectorStateSpace(1) for the velocity
	auto carSpace = std::make_shared<ob::CompoundStateSpace>();
	carSpace->setName("Car" + carSpace->getName());

	// create the 'configuration' space, an SE(2) space
	auto configurationSpace = std::make_shared<ob::SE2StateSpace>();
	// assign bounds to the SE(2) space
	ob::RealVectorBounds configurationBounds(2);
    configurationBounds.setLow(-10.0);
    configurationBounds.setHigh(10.0);
    configurationSpace->setBounds(configurationBounds);

	// create the 'velocity' space, a 1-dimensional real vector state space
	auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(1);
	// assign bounds to the velocity space
	ob::RealVectorBounds velocityBounds(1);
    velocityBounds.setLow(-1*vmax);
    velocityBounds.setHigh(vmax);
    velocitySpace->setBounds(velocityBounds);

	// add both spaces to the carSpace
	carSpace->addSubspace(configurationSpace, 1.0);
	carSpace->addSubspace(velocitySpace, 1.0);

	std::cout << "Created car state space" << std::endl;
	
	auto projection = new CarProjection(carSpace);
	carSpace->registerDefaultProjection(ob::ProjectionEvaluatorPtr(projection));
    
    // Create a control space
	// The first value is omega, the second value is acceleration
    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(carSpace, 2);

	std::cout << "Created car control space" << std::endl;
    
    // Set the bounds for the control space
    ompl::base::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1*omegamax);
    controlBounds.setHigh(0, omegamax);
	controlBounds.setLow(1, -1*vdotmax);
	controlBounds.setHigh(1, vdotmax);
    controlSpace->setBounds(controlBounds);

	std::cout << "Set bounds for control space" << std::endl;

	// Print out the control space bounds to confirm they were set correctly
	ob::RealVectorBounds new_bounds = controlSpace->getBounds();
    std::cout << "Omega: " << new_bounds.low[0] << ", " << new_bounds.high[0] << "; Vdot: " << new_bounds.low[1] << ", " << new_bounds.high[1] << std::endl;
    
    // SimpleSetup using the control space that was just defined and bounded
    oc::SimpleSetup ss(controlSpace);
    
    // Set state validity checker
	// This is causing segmentation faults right here
    ss.setStateValidityChecker([ss, obstacles](const ob::State* state) {
        return isStateValid(ss.getSpaceInformation().get(), obstacles, state);
    });

	std::cout << "Set state validity checker" << std::endl;
	
	// Create the state propagator
	auto si = ss.getSpaceInformation();
	oc::ODESolverPtr odeSolver (new oc::ODEBasicSolver<> (si, &carODE));

	std::cout << "Created the state propagator (ODEBasicSolver)... " << std::endl;
	
    // Set state propagator
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

	std::cout << "Set the state propagator" << std::endl;

    // Create a start state (x,y) = (-5,-5)
    ob::ScopedState<ob::CompoundStateSpace> start(carSpace);
	start->as<ob::SE2StateSpace::StateType>(0)->setX(-8.0);  // x
	start->as<ob::SE2StateSpace::StateType>(0)->setY(-5.0);  // y
	start->as<ob::SE2StateSpace::StateType>(0)->setYaw(0.0);  // theta
	start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;  // v
    
    // Create a goal state (x,y) = (5,5)
	ob::ScopedState<ob::CompoundStateSpace> goal(carSpace);
	goal->as<ob::SE2StateSpace::StateType>(0)->setX(8.0);  // x
	goal->as<ob::SE2StateSpace::StateType>(0)->setY(5.0);  // y
	goal->as<ob::SE2StateSpace::StateType>(0)->setYaw(0.0);  // theta
	goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;  // v
    
    // Set the start and goal states, with a goal state radius
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    // Set propagation step size and control duration limits so it stops yelling at me
    si->setPropagationStepSize(0.1);
	si->setMinMaxControlDuration(1, 10);
	std::cout << "Made the problem" << std::endl;
	return std::make_shared<oc::SimpleSetup>(ss);
}

void planCar(oc::SimpleSetupPtr & ss, int choice)
{
	if (choice == 1) {
		// RRT
		auto planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
		ss->setPlanner(planner);
		
		std::cout << "About to attempt to solve" << std::endl;
		ob::PlannerStatus solved = ss->solve(5.0);
		if (solved) {
			// Get the goal representation from the problem definition and inquire about the found path
			auto pdef = ss->getProblemDefinition();
			auto path = pdef->getSolutionPath();
			std::cout << "Found solution" << std::endl;
			path->print(std::cout);
			std::cout << "Solution as geometric path" << std::endl << std::endl;
            ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	} else if (choice == 2) {
		// KPIECE1
		auto planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
		ss->setPlanner(planner);
		ompl::base::PlannerStatus solved = ss->solve(5.0);
		if (solved) {
			// Get the goal representation from the problem definition and inquire about the found path
			auto pdef = ss->getProblemDefinition();
			auto path = pdef->getSolutionPath();
			std::cout << "Found solution" << std::endl;
			path->print(std::cout);
			std::cout << "Solution as geometric path" << std::endl << std::endl;
            ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	} else if (choice == 3) {
		// RG-RRT
		auto planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
		ss->setPlanner(planner);
		
		std::cout << "About to attempt to solve" << std::endl;
		ob::PlannerStatus solved = ss->solve(5.0);
		if (solved) {
			// Get the goal representation from the problem definition and inquire about the found path
			auto pdef = ss->getProblemDefinition();
			auto path = pdef->getSolutionPath();
			std::cout << "Found solution" << std::endl;
			path->print(std::cout);
			std::cout << "Solution as geometric path" << std::endl << std::endl;
            ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	} else {
		std::cout << "Invalid choice! Please use 1 (RRT), 2 (KPIECE1), or 3 (RG-RRT)" << std::endl;
	}
}

void benchmarkCar(oc::SimpleSetupPtr &/* ss */)
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
