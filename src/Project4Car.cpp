///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast
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

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Your projection for the car
		auto SE2State = state->as<ompl::base::SE2StateSpace::StateType>();
		projection(0) = SE2State->getX();
		projection(1) = SE2State->getY();
		projection(3) = SE2State->getYaw();
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
	// Useful https://ompl.kavrakilab.org/odeint.html
	// ompl::base::State values are translated into an iterable container of real values
	// q and qdot are vectors, like if you had called ompl::base::ScopedState::reals
	// the ompl::control::Control can be cast as an ompl::control::RealVectorControlSpace::ControlType to expose a vector property called values
	std::cout << "Inside carODE" << std::endl;
	const double * u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	
	std::cout << "About to access control values" << std::endl;
	
	const double omega = u[0];
	const double vdot = u[1];
	
	// The order of the elements of q and qdot depends on the base Space
	// For the CarSpace, it's Theta, X, Y, V
	const double theta = q[0];
	const double x = q[1];
	const double y = q[2];
	const double v = q[3];
	
	std::cout << "I be ode steppin" << std::endl;
	
	// Ensure that qdot is the same size as q and set all values to zero
	qdot.resize(q.size(), 0);
	
	// Assign the values of qdot according to our dynamical laws
	qdot[0] = omega;
	qdot[1] = v * cos(theta);
	qdot[2] = v * sin(theta);
	qdot[3] = vdot;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // Fill in the vector of rectangles with your street environment.
	Rectangle rect;
	
	rect.height = 1.0;
    rect.width = 1.0;
    rect.x = (0.0) - 0.5*rect.width;
    rect.y = (0.0) - 0.5*rect.height;
    obstacles.push_back(rect);
}

bool isStateValid(const ompl::control::SpaceInformation* spaceInformation, const std::vector<Rectangle> obstacles, const ompl::base::State* state) {
    // Perform collision checking or check if other constraints are satisfied
	std::cout << "About to check state validity" << std::endl;
	auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
	auto SO2State = compoundState->as<ompl::base::CompoundStateSpace::StateType>(0);
	auto realState = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);
    return spaceInformation->satisfiesBounds(state) && isValidPoint(realState->values[0], realState->values[1], obstacles);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // Create and setup the pendulum's state space, control space, validity checker, everything you need for planning.
    // Create a state space
	// This state space needs to have an SO2 component for the pendulum's rotation and a real component for angular velocity
    //auto carSpace = std::make_shared<ompl::base::SE2StateSpace>();
	auto carSpace = std::make_shared<ompl::base::CompoundStateSpace>();
	carSpace->setName("Car" + carSpace->getName());
	auto thetaSpace = std::make_shared<ompl::base::SO2StateSpace>();
	auto realSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3);
	carSpace->addSubspace(thetaSpace, 1.0);
	carSpace->addSubspace(realSpace, 1.0);
	
	//auto projection = new CarProjection(carSpace);
	//carSpace->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(projection));
    // TODO: ^ this causes seg fault
	
    // Set the bounds for the RealVector part of the state space
    ompl::base::RealVectorBounds realBounds(3);
    realBounds.setLow(-10);
    realBounds.setHigh(10);
	carSpace->as<ompl::base::RealVectorStateSpace>(1)->setBounds(realBounds);
    
    // Create a control space
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(carSpace, 2);
    
    // Set the bounds for the control space -- omega and acceleration (vdot)
    ompl::base::RealVectorBounds cBounds(2);
    cBounds.setLow(-10);
    cBounds.setHigh(10);
    cspace->setBounds(cBounds);
    
    // Here is a SimpleSetup
    ompl::control::SimpleSetup ss(cspace);
    
    // Set state validity checker
	// This is causing segmentation faults right here
    ss.setStateValidityChecker([ss, obstacles](const ompl::base::State* state) {
        return isStateValid(ss.getSpaceInformation().get(), obstacles, state);
    });
	
	// Create the state propagator
	auto si = ss.getSpaceInformation();
	ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &carODE));
	
    // Set state propagator
    ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // Create a start state (x,y) = (-5,-5)
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(carSpace);
	start->as<ompl::base::SO2StateSpace::StateType>(0)->value = 0.0;  // Theta
	start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = -5;  // X
	start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = -5;  // Y
	start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2] = 0.0;  // V
    
    // Create a goal state (x,y) = (5,5)
	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(carSpace);
	goal->as<ompl::base::SO2StateSpace::StateType>(0)->value = 0.0;  // Theta
	goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 5;  // X
	goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[1] = 5;  // Y
	goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[2] = 0.0;  // V
    
    // Set the start and goal states, with a goal state radius
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    // Set propagation step size and control duration limits so it stops yelling at me
    si->setPropagationStepSize(0.1);
	si->setMinMaxControlDuration(1, 10);
	std::cout << "Made the problem" << std::endl;
	return std::make_shared<ompl::control::SimpleSetup>(ss);
}

void planCar(ompl::control::SimpleSetupPtr & ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
	if (choice == 1) {
		// RRT
		auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
		ss->setPlanner(planner);
		
		std::cout << "About to attempt to solve" << std::endl;
		ompl::base::PlannerStatus solved = ss->solve(5.0);
		if (solved) {
			// Get the goal representation from the problem definition and inquire about the found path
			auto pdef = ss->getProblemDefinition();
			auto path = pdef->getSolutionPath();
			std::cout << "Found solution" << std::endl;
			path->print(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	} else if (choice == 2) {
		// KPIECE1
		auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
		ss->setPlanner(planner);
		ompl::base::PlannerStatus solved = ss->solve(5.0);
		if (solved) {
			// Get the goal representation from the problem definition and inquire about the found path
			auto pdef = ss->getProblemDefinition();
			auto path = pdef->getSolutionPath();
			std::cout << "Found solution" << std::endl;
			path->print(std::cout);
		} else {
			std::cout << "No solution found" << std::endl;
		}
	} else if (choice == 3) {
		// RG-RRT
		
	} else {
		std::cout << "Invalid choice! Please use 1 (RRT), 2 (KPIECE1), or 3 (RG-RRT)" << std::endl;
	}
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
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

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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
