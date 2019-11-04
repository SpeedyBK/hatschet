//
// Created by nfiege on 17/10/19.
//

#ifndef HATSCHET_MODULOQSCHEDULER_H
#define HATSCHET_MODULOQSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
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
		 * enable/disable debug couts
		 * @param q
		 */
		void setQuiet(bool q) { this->quiet = q; }
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
		 * print contents of MRT
		 */
		void print();
		/*!
		 *
		 * @param v vertex
		 * @return all modulo slots where v appears
		 */
		std::vector<int> getModuloSlots(Vertex* v);
		/*!
		 * get height of MRT in the given column
		 * @param res resource
		 * @param column
		 * @return
		 */
		int getHeight(const Resource* res, int column);
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


	class ModuloQScheduler : public SchedulerBase, public ILPSchedulerBase, public RationalIISchedulerLayer {
	public:
		/*!
		 *
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		ModuloQScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * enable/disable debug couts
		 * @param q
		 */
		void setQuiet(bool q) { this->quiet = q; }
		/*!
		 *
		 * @param maxIt
		 */
		void setMaxLatencySequenceIterations(int maxIt) { this->maxSequenceIterations = maxIt; }
		/*!
		 * To Be Updated: II has to be rational or a vector for this scheduler to work
		 * @return
		 */
		double getII() override { return this->II;}
		/*!
		 *
		 * @return S and M (samples and modulo)
		 */
		std::pair<int,int> getSM();
		/*!
		 * Main schedule function
		 */
		void schedule() override;
		/*!
		 * Calculate all possible initiation intervals for the given M and S values
		 * @param M
		 * @param S
		 * @return
		 */
		static std::vector<std::vector<int>> getAllInitiationIntervals(int M, int S);
		/*!
		 * compute intervals between samples for the initiation intervals sequence
		 * @param initIntervals
		 * @param M modulo
		 * @return
		 */
		static std::vector<int> getLatencySequenceFromInitiationIntervals(std::vector<int> &initIntervals, int M);
		/*!
		 *
		 * @return initiation intervals for the latency sequence
		 */
		std::vector<int> getInitiationIntervals() const { return this->initiationIntervals; }
		/*!
		 *
		 * @return latency sequences for which no solution was found
		 */
		std::vector<std::vector<int>> getDiscardedInitiationIntervals() const { return this->discardedInitiationIntervals; }
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
		 * binding function not implemented yet
		 * @return
		 */
		vector<std::map<const Vertex*,int> > getRationalIIBindings() override { throw HatScheT::Exception("ModuloQScheduler::getRationalIIBindings not implemented yet"); }
		/*!
		 * sort initiation intervals by the variance from S/M
		 * @param unsortedInitIntervals
		 * @param S samples
		 * @param M modulo
		 * @return
		 */
		static std::vector<std::vector<int>> getInitIntervalQueue(std::vector<std::vector<int>>& unsortedInitIntervals, int S, int M);
	protected:
		/*!
		 * not needed
		 */
		void constructProblem() override {}
		/*!
		 * not needed
		 */
		void resetContainer() override {}
		/*!
		 *
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
		 * maximum number of latency sequences for which a solution is attempted to be found
		 */
		int maxSequenceIterations;
		/*!
		 * contains latency sequences for which no solution was found
		 */
		std::vector<std::vector<int>> discardedInitiationIntervals;
		/*!
		 * no couts if this is true
		 */
		bool quiet;
		/*!
		 * try scheduling and report if a solution was found
		 * @return
		 */
		bool scheduleAttempt();
		/*!
		 * finds a valid non-rectangular MRT for this->latencySequence
		 */
		void setMRT();
		/*!
		 * all possible init interval sequences for the given S/M
		 */
		std::vector<std::vector<int>> allInitiationIntervals;
		/*!
		 * initiation intervals for the found schedule
		 */
		std::vector<int> initiationIntervals;
		/*!
		 * the minimum interger II that is possible
		 */
		int integerMinII;
		/*!
 		* @brief the s value for the iteration start
 		*/
		int S;
		/*!
		 * @brief the m value for the iteration start
		 */
		int M;
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
		//std::map<const Vertex*, ScaLP::Variable> k, row, time;
		std::map<const Vertex*, ScaLP::Variable> k, time;
	};
}



#endif //HATSCHET_MODULOQSCHEDULER_H
