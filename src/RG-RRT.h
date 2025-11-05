///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Henry Prendergast and Max Kuhlman
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <vector>

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
			
			// Get the number of propagation steps used for reachability controls
			int getReachControlSteps() const {
				return reachControlSteps_;
			}
			
			// Set the number of propagation steps used for reachability controls
			void setReachControlSteps(int reachControlSteps) {
				reachControlSteps_ = reachControlSteps;
			}
			
			// Get the index of the control dimension used for reachability
			int getReachControlDimIdx() const {
				return reachControlDimIdx_;
			}
			
			// Set the index of the control dimension used for reachability
			void setReachControlDimIdx(int reachControlDimIdx) {
				reachControlDimIdx_ = reachControlDimIdx;
			}
			
			// Write a PlannerData object
			void getPlannerData(base::PlannerData &data) const override;
			
			// Template for creating nearest neighbors datastructures
			template <template <typename T> class NN>

			// Set different nearest neighbors datastructure
			void setNearestNeighbors() {
				if (nn_ && nn_->size() != 0) {
					OMPL_WARN("Calling setNearestNeighbors will clear all states!");  // This seems like a ! sort of warning
				}
				clear();
				nn_ = std::make_shared<NN<Motion *>>();
				rn_ = std::make_shared<NN<Motion *>>(); // TESTING
				setup();
			}
			
			void setup() override;
			
		protected:
			const std::vector<std::string> names = {"Abeline", "Amy", "Alex", "Alexander", "Beth", "Bill", "Bob", "Cathy", "Claire", "Dan", "Danielle", "Emily", "Edward", "Frank", "Frahanco", "Fred", "Faith", "Gerald", "Gertrude", "Gemma", "Henry", "Hank", "Helena", "Hannah", "Ian", "Isabella", "Jake", "John", "Jerry", "Jean", "Kyle", "Karl", "Kris", "Louise", "Larry", "Max", "May", "Mary", "Nancy", "Neel", "Owen", "Ophelia", "Pat"};
			
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
				
				// Give this Motion a human-readable identifier, such as "Bob"
				// There's a chance that it shares this idenfitier with other Motions, so you should also check another property
				std::string name;
				
				// Vector of reachable States (represented by State *)
				std::vector<base::State*> reachable;
				
				// Vector of Controls that got us to the reachable States
				std::vector<Control*> reachableControls;
			};
			
			// Free the memory allocated by this planner
			void freeMemory();

			// Add reachable states to a motion
			void addReachableStates(Motion *nmotion);
			
			double distanceFunction(const Motion *a, const Motion *b) const {
				return si_->distance(a->state, b->state);
			}
			
			// State sampler
			base::StateSamplerPtr sampler_;
			
			// Control sampler
			DirectedControlSamplerPtr controlSampler_;
			
			// The base::SpaceInformation case as a control::SpaceInformation
			const SpaceInformation *siC_;
			
			// Duration over which that we apply controls for reachibility analysis
			int reachControlSteps_{3};
			
			// Control subspace index for the reachability estimation
			int reachControlDimIdx_{0};
			
			// A nearest-neighbors datastructure containing the tree of Motions
			std::shared_ptr<NearestNeighbors<Motion *>> nn_;
			// A nearest-neighbors datastructure containing the tree of reachable motions
			std::shared_ptr<NearestNeighbors<Motion *>> rn_; // TESTING
			
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
