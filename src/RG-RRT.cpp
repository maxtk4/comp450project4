///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast and Max Kuhlman
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>
#include <cmath>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// TODO: Implement RGRRT as described

// For pendulum:
// Pick 11 evenly-spaced values for the control torque between the torque limits

// For car:
// Pick 11 evenly-spaced values for control u0 (omega, steering speed) (you can ignore u1 (vdot, linear acceleration))

// The control space should already have bounds to the control values, so we just need to know which control to estimate reachability for
// For this specific project, it happens to be the zeroth control subspace, but it would be nice to input that to the planner to make it more robust

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT") {
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates, "0,1");
	Planner::declareParam<int>("reach_control_steps", this, &RGRRT::setReachControlSteps, &RGRRT::getReachControlSteps, "0:5:10");  // change this to be intelligent
	Planner::declareParam<int>("reach_control_dim_idx", this, &RGRRT::setReachControlDimIdx, &RGRRT::getReachControlDimIdx, "0:1:2");  // change this to be intelligent
}

ompl::control::RGRRT::~RGRRT() {
	freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!rn_) // TESTING
        rn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this)); // TESTING
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    rn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); }); // TESTING
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (rn_)
        rn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
			// TOOD: Add something to remove the states and controls in the vectors
            delete motion;
        }
    }
    if (rn_)
    {
        std::vector<Motion *> motions;
        rn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
	
	// TODO: Add some representation of the reachable states that are attached to each motion

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

void ompl::control::RGRRT::addReachableStates(Motion *motion)
{
	//std::cout << "Adding reachable states to motion " << motion->name << " with state angle: " << motion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
    // Here we are assuming that the control space is actually (or is castable to) a RealVectorControlSpace
    ompl::control::RealVectorControlSpace * ctrlSpace = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>();  
    
    ompl::base::RealVectorBounds ctrlBounds = ctrlSpace->getBounds();
    double controlLow = ctrlBounds.low[reachControlDimIdx_];
    double controlDelta = (ctrlBounds.high[reachControlDimIdx_] - controlLow) / 10.0;
    
    for (int i = 0; i < 11; i++) {
        double controlValue = controlLow + controlDelta * i;
        ompl::control::RealVectorControlSpace::ControlType * reachControl = siC_->allocControl()->as<ompl::control::RealVectorControlSpace::ControlType>();  // I'm a little worried about this going out of scope
        reachControl->values[reachControlDimIdx_] = controlValue;
        // std::cout << "Control value: " << reachControl->values[reachControlDimIdx_]<< std::endl;
        // The controlValues are properly spaced out--11 points, including the bounds
        
        // Options we have in siC_:
        // propagate(const base::State *state, const Control *control, int steps, std::vector<base::State * > &result, bool alloc)
        // propagateWhileValid(const base::State *state, const Control *control, int steps, base::State *result)
        // propagateWhileValid(const base::State *state, const Control *control, int steps, std::vector<base::State * > &result, bool alloc)
    
        ompl::base::State * result = si_->allocState();
        
        // This should propagate each of the evenly sampled control to add a point to the array of reachable points
        // What does this do about the velocity of the car?
        int validSteps = siC_->propagateWhileValid(motion->state, reachControl, reachControlSteps_, result);
		
		//if (std::fabs(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value - 1.5708) < 0.01) {
		//	std::cout << "\033[31m" << "Result has angle 1.5708" << "\033[0m" << std::endl;
		//}
        
        if (validSteps == reachControlSteps_) {
            // if the propagation was successful, add to the nmotion object
			//std::cout << "Checking that added state is accessible" << std::endl;
			//std::cout << i << "st/nd/rd/th reachable state result theta: " << result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
            motion->reachable.push_back(result);
			motion->reachableControls.push_back(reachControl);
			//std::cout << "motion->reachable.back() theta: " << motion->reachable.back()->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
        } else {
			si_->freeState(result);
			siC_->freeControl(reachControl);
		}
    }
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
		motion->name = names[(int)(rng_.uniform01() * names.size())] + std::to_string((int)(rng_.uniform01() * 100));
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);

        addReachableStates(motion);
		
		//std::cout << "Number of reachable states for this start: " << motion->reachable.size() << std::endl;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
	rmotion->name = names[(int)(rng_.uniform01() * names.size())] + std::to_string((int)(rng_.uniform01() * 100));
    //base::State *rstate = rmotion->state;
    //Control *rctrl = rmotion->control;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
			std::cout << "Sampling goal" << std::endl;
			goal_s->sampleGoal(rmotion->state);
		}
        else {
			std::cout << "Sampling random" << std::endl;
            sampler_->sampleUniform(rmotion->state);
		}
		
		std::cout << "Sampled motion has state angle " << rmotion->state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << " and velocity " << rmotion->state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] << std::endl;
		
		// From the actual paper:
		// More specifically, the NEARESTSTATE(xrand,T) function compares the distance from the random sample not only to the nodes, 
		// but also to the points within their reachable sets. If the closest Reachable point is closer to the sample than the closest node of the tree, 
		// then both this reachable point, xrnear, and its corresponding parent node, xnear, are returned. 
		// Otherwise, if the closest node of the tree is nearer to the sample than any Reachable point, the function returns an empty point pair, 
		// in which case the RG-RRT throws this sample away, and draws a new sample from the state space and repeats the process.

		//std::cout << "After initial sampling, " << rmotion->name << " rmotion->state theta is: " << rmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;


		// nn_ is a NearestNeighbors that was constructed using Motions, so it requires a Motion to calculate distance
        Motion *nmotion = nn_->nearest(rmotion);
		
		if (nmotion->reachable.size() > 0) {  // Make sure this isn't a dead end

			auto distNearestSampled = si_->distance(nmotion->state, rmotion->state);
			
			std::cout << nmotion->name << " nmotion->state theta is: " << nmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << ", velocity: " << nmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] << ", distance = " << distNearestSampled << std::endl;


			// Check if there is a reachable state which is closer than the nearest neighbor state
			auto distReachableSampled = -1.0;
			int nrIdx = -1;
			for (int r = 0; r < (int)nmotion->reachable.size(); r++) {
				auto distReachSampled = si_->distance(nmotion->reachable[r], rmotion->state);
				std::cout << nmotion->name << " nmotion " << r << "st/nd/rd/th reachable state angle: " << nmotion->reachable[r]->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << ", velocity: " << nmotion->reachable[r]->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] << ", control: " << nmotion->reachableControls[r]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << " --> distance = " << distReachSampled << std::endl;
				if (distReachableSampled == -1.0 || distReachSampled < distReachableSampled) {
					// This reachable state is closer to the sampled point than the currently-held one
					distReachableSampled = distReachSampled;
					nrIdx = r;
				}
			}
			// Now nrIdx has the index of the reachable state that's closest to the sampled point
			// If this motion has no reachable states, we need to catch that (nrIdx is still -1)
			if (nrIdx != -1 && distReachableSampled < distNearestSampled) {
				// We want to use this reachable point to expand the tree
				//std::cout << "Found a good reachable state at index " << nrIdx << std::endl;
				//std::cout << "rmotion state angle: " << rmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
				//std::cout << "nmotion state angle: " << nmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
				//std::cout << "nmotion reachable size: " << nmotion->reachable.size() << std::endl;
				//std::cout << "nmotion front reachable state angle: " << nmotion->reachable.front()->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
				//std::cout << nmotion->name << " nmotion nrIdx reachable state angle: " << nmotion->reachable[nrIdx]->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
				//std::cout << nmotion->name << " nmotion nrIdx reachable control: " << nmotion->reachableControls[nrIdx]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << std::endl;
				//rmotion->state = nmotion->reachable[nrIdx];
				//rmotion->control = nmotion->reachableControls[nrIdx];
				// ^ Using these lines would make nmotion->reachable[nrIdx] become rmotion->state sometimes. Not sure why
				std::cout << "Updating rmotion with closer reachable state" << std::endl;
				si_->copyState(rmotion->state, nmotion->reachable[nrIdx]);
				siC_->copyControl(rmotion->control, nmotion->reachableControls[nrIdx]);
			} else {
				// Otherwise, continue to the next iteration of the loop
				std::cout << "nmotion " << nmotion->name << " is closer to the sampled point than any reachable points" << std::endl;
				continue;
			}
		} else {
			//std::cout << "The motion didn't have any reachable states" << std::endl;
			continue;
		}
				
		//std::cout << "rstate theta before controlSampler_: " << rstate->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
		//std::cout << rmotion->name << " rmotion->state theta is now: " << rmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
		
		// Next, according to the paper:
		// Upon identifying a suitable node for expansion, the RG-RRT extends the tree from the node. 
		// The SOLVEINPUT(xnear,xrnear,xrand,T) function can either 
		// search for a consistent action that drives the system from the state towards the sample or 
		// returns the control associated with the reachable point, xrnear. 
		// In either case, lines 10 and 11 in Algorithm 1 identify a collision-free trajectory 
		// to a state within R(xnear) that obeys the system's differential constraints.
		// Figure 3(b) demonstrates this step where we have extended the node to the point, xnew=xrnear, 
		// in its reachable set that was the nearest to the sample.

		/* Sample a random control that attempts to go towards the random state, and also sample a control duration */
		// From the docs: arguments are:
		// Control * control - Where to store the sampled control
		// const Control * previous - The previous control; useful for planners with a sense of directivity
		// const base::State * source - The starting point for this control
		// base::State * dest - The destination that this control should strive to travel to; modified to match the state reached
		// NOTE: motion is checked for validity
		// NOTE: returns the duration that the control should be applied for
		
		//unsigned int cd = controlSampler_->sampleTo(rmotion->control, nmotion->control, nmotion->state, rmotion->state);
		unsigned int cd = reachControlSteps_;
		// This could probably be changed to use the control that generated the reachable state
		//std::cout << "rstate theta after controlSampler_: " << rstate->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
		//std::cout << "rmotion->state theta after controlSampler_: " << rmotion->state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << std::endl;
		
		if (cd >= siC_->getMinControlDuration())
		{
			// This control creates valid movement for at least a little while; add a motion
			auto *motion = new Motion(siC_);
			motion->name = names[(int)(rng_.uniform01() * names.size())] + std::to_string((int)(rng_.uniform01() * 100));
			si_->copyState(motion->state, rmotion->state);
			siC_->copyControl(motion->control, rmotion->control);
			motion->steps = cd;
			motion->parent = nmotion;

			nn_->add(motion);
			double dist = 0.0;
			bool solv = goal->isSatisfied(motion->state, &dist);
			if (solv)
			{
				approxdif = dist;
				solution = motion;
				break;
			}
			if (dist < approxdif)
			{
				approxdif = dist;
				approxsol = motion;
			}
			
			// We didn't get to the goal, so now we can find some reachable states
			addReachableStates(motion);
			//std::cout << "Number of reachable states for new motion: " << motion->reachable.size() << std::endl;
			std::cout << "Added new motion " << motion->name << " with state angle " << motion->state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(0)->value << ", velocity " << motion->state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[0] << std::endl;
		}
    }
	
	//std::cout << "Collecting solutions" << std::endl;

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}