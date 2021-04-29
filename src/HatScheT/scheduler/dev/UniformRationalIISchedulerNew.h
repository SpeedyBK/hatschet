//
// Created by nfiege on 08/04/20.
//

#ifndef HATSCHET_UNIFORMRATIONALIISCHEDULERNEW_H
#define HATSCHET_UNIFORMRATIONALIISCHEDULERNEW_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT {
	/*!
	 * This is an improved version of the UniformRationalIIScheduler
	 * which uses less variables for the same problem formulation
	 */
	class UniformRationalIISchedulerNew : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		/*!
		 * Constructor
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		UniformRationalIISchedulerNew(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * print the rational II modulo schedule
		 */
		void printScheduleToConsole();
		/*!
		 * @brief print the MRTs of all resources after rational II scheduling and binding
		 */
		void printBindingToConsole();

	protected:
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by RationalIISchedulerLayer::schedule
		 *
		 */
		void scheduleIteration() override;
		/*!
		 * constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
		 */
		void constructProblem() override;
		/*!
		 * setObjective currently asap
		 */
		void setObjective() override;
		/*!
		 * reset containers
		 */
		void resetContainer() override;

	private:
		/*!
		 * initiation intervals for the found schedule
		 */
		std::vector<int> initiationIntervals;
		/*!
		 * minimum deltas according to the latency sequence
		 */
		std::map<unsigned int,unsigned int> deltaMins;
		/*!
		 * for each distance in the graph, calculate the minimum delta according to the latency sequence
		 */
		void calcDeltaMins();
		/*!
		 * set the resource constranints / often referred to as modulo reservation table (MRT)
		 */
		void setResourceConstraints();
		/*!
		 * set dependency contraints
		 */
		void setGeneralConstraints();
		/*!
		 * set modulo constraints
		 */
		void setModuloConstraints();
		/*!
		 * the ILP variables of the operations in the input graph
		 */
		void fillTContainer();
		/*!
		 * the ILP variables of the modulo slots of all operations in the input graph
		 */
		void fillBContainer();
		/*!
		 * fill interface to pass values to next step in the tool flow after solving
		 */
		void fillSolutionStructure();
		/*!
		 * calc distance between two samples in clock cycles
		 * @param d edge distance
		 * @param startIndex sample index
		 * @return
		 */
		int getSampleDistanceAsInt(int d, int startIndex);
		/*!
		 * container for ILP variables
		 */
		std::map<Vertex*,ScaLP::Variable> tVariables;
		/*!
		 * container for ILP variables
		 * map Vertex* -> b-Matrix
		 * first index of b-Matrix: sample index in {0, ..., #samples-1}
		 * second index of b-Matrix: modulo slot
		 */
		std::map<const Vertex*,std::vector<ScaLP::Variable>> bVariables;
		/*!
		 * calc tau hat in R3
		 * @param s sample index
		 * @param oldModuloslot modulo slot tau (without hat)
		 * @return tau hat
		 */
		int getNewModuloslot(int s, int oldModuloslot);
	};
}


#endif //HATSCHET_UNIFORMRATIONALIISCHEDULERNEW_H
