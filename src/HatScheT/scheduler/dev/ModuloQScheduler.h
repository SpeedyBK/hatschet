//
// Created by nfiege on 17/10/19.
//

#ifndef HATSCHET_MODULOQSCHEDULER_H
#define HATSCHET_MODULOQSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <HatScheT/utility/subgraphs/SCC.h>
#include <vector>

namespace HatScheT {
	class ModuloQMRT {
	public:
		/*!
		 * default constructor
		 */
		ModuloQMRT();
		/*!
		 * Construct a rectangular MRT
		 * @param rm resource model
		 * @param II number of columns
		 */
		ModuloQMRT(ResourceModel &rm, unsigned int II);
		/*!
		 * specify height of MRT at the given column number
		 * @param res
		 * @param column
		 * @param height
		 */
		void specifyColumnHeight(const Resource* res, unsigned int column, unsigned int height);
		/*!
		 * set resource model for this MRT
		 * @param rm
		 */
		void setResourceModelAndII(ResourceModel &rm, unsigned int II);
		/*!
		 * insert a vertex into this MRT
		 * @param v address of the vertex
		 * @param moduloSlot column where it should be inserted
		 * @return if it was successfully inserted, i.e. MRT is full at this slot => return false
		 */
		bool insertVertex(Vertex* v, unsigned int moduloSlot);
		/*!
		 * remove all instances of this vertex from MRT
		 * @param v
		 * @return if it was successfully removed
		 */
		bool removeVertex(Vertex* v);
		/*!
		 * @param m modulo slot
		 * @param res resource
		 * @return a list of vertices in modulo slot m
		 */
		std::list<Vertex*> getVerticesInModuloSlot(int m, const Resource* res);
		/*!
		 * print contents of MRT
		 */
		void print() const;
		/*!
		 *
		 * @param v vertex
		 * @return all modulo slots where v appears
		 */
		std::vector<int> getModuloSlots(Vertex* v) const;
		/*!
		 * get height of MRT in the given column
		 * @param res resource
		 * @param column
		 * @return
		 */
		int getHeight(const Resource* res, int column) const;
		/*!
		 * rotate MRT left, i.e. column 0 <- column 1; column 1 <- column 2; ... ; column N-1 <- column 0
		 */
		void rotateLeft();
		/*!
		 *
		 * @return maximum number of rotations for the given II and resource model
		 */
		unsigned long getMaxNumberOfRotations();

	private:
		/*!
		 * no couts if this is true
		 */
		bool quiet;
		/*!
		 * this is where everything is stored
		 */
		std::map<const Resource*,std::vector<std::vector<Vertex*>>> mrt;
		/*!
		 * resource model for this MRT
		 */
		ResourceModel* rm;
		/*!
		 * number of columns
		 */
		unsigned int II;
		/*!
		 * track number of rotations from rotateLeft() function
		 */
		std::map<const Resource*,int> rotations;
	};


	class ModuloQScheduler : public ILPSchedulerBase, public RationalIISchedulerLayer {
	public:
		/*!
		 *
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		ModuloQScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * To Be Updated: II has to be rational or a vector for this scheduler to work
		 * @return
		 */
		double getII() override { return this->II; }
		/*!
		 * non-critical resources might have non-rectangular MRT shapes
		 * @return vector with column heights
		 */
		std::map<Resource*,std::vector<int>> getMRTShape() const;
		/*!
		 *
		 * @return initiation intervals for the latency sequence
		 */
		std::vector<int> getInitiationIntervals() const { return this->initiationIntervals; }
		/*!
		 * modulo operation with non-negative result
		 * @param a
		 * @param b
		 * @return
		 */
		static inline int mod(int a, int b) {
			int m = a % b;
			return m >= 0 ? m : m + b;
		}
		/*!
		 * finds a valid non-rectangular MRT for a given resource model and a initiation intervals
		 */
		static void setMRT(ModuloQMRT &mrt, ResourceModel &resourceModel, std::vector<int> &initiationIntervals, int samples, int modulo, bool quiet);
	protected:
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
		void scheduleIteration() override;
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		/*!
		 * not needed
		 */
		void resetContainer() override {}
		/*!
		 * set ILP objective
		 */
		void setObjective() override;
		/*!
		 * reset solver
		 */
		void setUpSolverSettings();
		/*!
		 * create variables
		 */
		void constructDecisionVariables();
		/*!
		 * create constraints
		 */
		void constructConstraints();

	private:
		/*!
		 * compute and sort all initiation intervals
		 */
		void setInitiationIntervals();
		/*!
		 * try scheduling and report if a solution was found
		 * @return
		 */
		bool scheduleAttempt();
		/*!
		 * initiation intervals for the found schedule
		 */
		std::vector<int> initiationIntervals;
		/*!
		 * complete modulo reservation table containing all samples
		 */
		ModuloQMRT mrt;
		/*!
		 * variables for resource constraints
		 */
		std::vector<std::map<const Vertex*, ScaLP::Variable>> a;
		/*!
		 * schedule time variables
		 */
		std::map<const Vertex*, ScaLP::Variable> k, time;
	};
}



#endif //HATSCHET_MODULOQSCHEDULER_H
