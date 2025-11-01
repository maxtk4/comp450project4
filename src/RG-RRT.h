///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast and Max Kuhlman
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ompl
{
    namespace control
    {
        // TODO: Implement RGRRT as described

        class RGRRT : public base::Planner
        {
		public:
			RGRRT(const SpaceInformationPtr &si);
			
			~RGRRT() override;
			
			// Solve a motion planning problem
			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
			
			// Clear datastructures
			void clear() override;
			
			// Set the goal bias
			void setGoalBias(double goalBias) {
				goalBias_ = goalBias;
			}
			
			// Get the goal bias
			double getGoalBias() const {
				return goalBias_;
			}
			
			// Get the state of addIntermediateStates_
			bool getIntermediateStates() const {
				return addIntermediateStates_;
			}
			
			// Set the state of addIntermediateStates_
			void setIntermediateStates(bool addIntermediateStates) {
				addIntermediateStates_ = addIntermediateStates;
			}
			
			// Write a PlannerData object
			void getPlannerData(base::PlannerData &data) const override;
			
			// Set a different nearest neighbors datastructure
			template <template <typename T> class NN>
			void setNearestNeighbors() {
				if (nn_ && nn_->size() != 0) {
					OMPL_WARN("Calling setNearestNeighbors will clear all states!");  // This seems like a ! sort of warning
				}
				clear();
				nn_ = std::make_shared<NN<Motion *>>();
				setup();
			}
			
			void setup() override;
			
		protected:
			class Motion {
			public:
				Motion() = default;
				
				Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {
				}
				
				~Motion() = default;
				
				// The state contained by the motion
				base::State *state{nullptr};
				
				// The control contained by the motion
				Control *control{nullptr};
				
				// The number of steps that the control is applied for
				unsigned int steps{0};
				
				// The parent motion in the exploration tree
				Motion *parent{nullptr};
			};
			
			// Free the memory allocated by this planner
			void freeMemory();
			
			double distanceFunction(const Motion *a, const Motion *b) const {
				return si_->distance(a->state, b->state);
			}
			
			// State sampler
			base::StateSamplerPtr sampler_;
			
			// Control sampler
			DirectedControlSamplerPtr controlSampler_;
			
			// The base::SpaceInformation case as a control::SpaceInformation
			const SpaceInformation *siC_;
			
			// A bearest-neighbors datastructure containing the tree of Motions
			std::shared_ptr<NearestNeighbors<Motion *>> nn_;
			
			// The fraction of the time the goal is picked as the state to expand towards
			double goalBias_{0.05};
			
			// Flag indicating whether intermediate states are added to the built tree of Motions
			bool addIntermediateStates_{false};
			
			// Random number generator
			RNG rng_;
			
			// The most recent goal motion. Used for PlannerData computation
			Motion *lastGoalMotion_{nullptr};
		};

    }  // namespace control 
}  // namespace ompl

#endif
