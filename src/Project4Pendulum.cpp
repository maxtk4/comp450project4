///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/ODESolver.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator {
public:
    PendulumProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override {
        // The dimension of the projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override {
        // Your projection for the pendulum
		auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
		projection(0) = compoundState->as<ompl::base::SO2StateSpace::StateType>(0)->value;
		projection(1) = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0];
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
                 ompl::control::ODESolver::StateType & qdot) {
	const double g = 9.81;  // m / s^2
	// Useful https://ompl.kavrakilab.org/odeint.html
	// ompl::base::State values are translated into an iterable container of real values
	// q and qdot are vectors, like if you had called ompl::base::ScopedState::reals
	// the ompl::control::Control can be cast as an ompl::control::RealVectorControlSpace::ControlType to expose a vector property called values
	const double * u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	
	const double torque = u[0];
	
	// The order of the elements of q and qdot depends on the base Space
	// In this case, I think it's 0 = theta, 1 = omega
	const double theta = q[0];
	const double omega = q[1];
	
	// Ensure that qdot is the same size as q and set all values to zero
	qdot.resize(q.size(), 0);
	
	// Assign the values of qdot according to our dynamical laws
	qdot[0] = omega;
	qdot[1] = -g * cos(theta) + torque;
}

bool isStateValid(const ompl::control::SpaceInformation* spaceInformation, const ompl::base::State* state) {
    // Perform collision checking or check if other constraints are satisfied
    return spaceInformation->satisfiesBounds(state);
}

ompl::control::SimpleSetupPtr createPendulum(double torque) {
    // Create and setup the pendulum's state space, control space, validity checker, everything you need for planning.
    // Create a state space
	// This state space needs to have an SO2 component for the pendulum's rotation and a real component for angular velocity
    auto pendulumSpace = std::make_shared<ompl::base::CompoundStateSpace>();
	pendulumSpace->setName("Pendulum" + pendulumSpace->getName());
	auto thetaSpace = std::make_shared<ompl::base::SO2StateSpace>();
	auto omegaSpace = std::make_shared<ompl::base::RealVectorStateSpace>(1);
	pendulumSpace->addSubspace(thetaSpace, 1.0);
	pendulumSpace->addSubspace(omegaSpace, 1.0);
	auto projection = new PendulumProjection(pendulumSpace);
	pendulumSpace->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(projection));
	//pendulumSpace->lock();
    
    // Set the bounds for the omega part of the space
    ompl::base::RealVectorBounds omegaBounds(1);
    omegaBounds.setLow(-10);
    omegaBounds.setHigh(10);
	pendulumSpace->as<ompl::base::RealVectorStateSpace>(1)->setBounds(omegaBounds);
    
    // Create a control space
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(pendulumSpace, 1);
    
    // Set the bounds for the control space -- torque
    ompl::base::RealVectorBounds torqueBounds(1);
    torqueBounds.setLow(-torque);
    torqueBounds.setHigh(torque);
    cspace->setBounds(torqueBounds);
    
    // Here is a SimpleSetup
    ompl::control::SimpleSetup ss(cspace);
    
    // Set state validity checker
	// This was causing seg faults when it captured ss&
    ss.setStateValidityChecker([ss](const ompl::base::State* state) {
        return isStateValid(ss.getSpaceInformation().get(), state);
    });
	
	// Create the state propagator
	auto si = ss.getSpaceInformation();
	ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &pendulumODE));
	
    // Set state propagator
    ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // Create a start state (-pi/2, 0)
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(pendulumSpace);
	start->as<ompl::base::SO2StateSpace::StateType>(0)->value = -3.141592/2.0;
    start->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
    
    // Create a goal state (+pi/2, 0)
	ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(pendulumSpace);
	goal->as<ompl::base::SO2StateSpace::StateType>(0)->value = 3.141592/2.0;
    goal->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
    
    // Set the start and goal states, with a goal state radius
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    // Set propagation step size and control duration limits so it stops yelling at me
    si->setPropagationStepSize(0.1);
	si->setMinMaxControlDuration(1, 10);
	
	return std::make_shared<ompl::control::SimpleSetup>(ss);
}

void planPendulum(ompl::control::SimpleSetupPtr & ss , int choice) {
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
	
	if (choice == 1) {
		// RRT
		auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
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

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */) {
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */) {
    int choice;
    do {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do {
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
    if (choice == 1) {
        int planner;
        do {
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
