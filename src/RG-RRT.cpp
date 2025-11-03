///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast and Max Kuhlman
//////////////////////////////////////

#include "RG-RRT.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

#include <ompl/control/spaces/RealVectorControlSpace.h>

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
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
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

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
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
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

		// If we don't care about intermediate states, we can ignore this
        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;  // 
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
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
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
				// cd is less than the minimum control duration; presumably, this control results in an invalid state
				// This motion is no good, so I guess we'll just discard it
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
		// Continue here if we don't care about intermediate states
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                // This control creates valid movement for at least a little while; add a motion
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);  // I might move this to after we add the reachable states
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
				// Here we are assuming that the control space is actually (or is castable to) a RealVectorControlSpace
				ompl::control::RealVectorControlSpace * ctrlSpace = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>();  
				
				ompl::base::RealVectorBounds ctrlBounds = ctrlSpace->getBounds();
				double controlLow = ctrlBounds.low[reachControlDimIdx_];
				double controlDelta = (ctrlBounds.high[reachControlDimIdx_] - controlLow) / 10.0;
				
				for (int i = 0; i < 11; i++) {
					double controlValue = controlLow + controlDelta * i;
					ompl::control::RealVectorControlSpace::ControlType * reachControl = siC_->allocControl()->as<ompl::control::RealVectorControlSpace::ControlType>();  // I'm a little worried about this going out of scope
					reachControl->values[reachControlDimIdx_] = controlValue;
					//std::cout << "Control value: " << reachControl->values[reachControlDimIdx_]<< std::endl;
					// The controlValues are properly spaced out--11 points, including the bounds
					
					// Options we have in siC_:
					// propagate(const base::State *state, const Control *control, int steps, std::vector<base::State * > &result, bool alloc)
					// propagateWhileValid(const base::State *state, const Control *control, int steps, base::State *result)
					// propagateWhileValid(const base::State *state, const Control *control, int steps, std::vector<base::State * > &result, bool alloc)
				
					ompl::base::State * result = si_->allocState();
					
					int validSteps = siC_->propagateWhileValid(motion->state, reachControl, reachControlSteps_, result);
					
					if (validSteps == reachControlSteps_) {
						motion->reachable.push_back(result);
					}
				}
				
				// Now the motion has a bunch of reachable states associated with it
            }
        }
    }

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
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}