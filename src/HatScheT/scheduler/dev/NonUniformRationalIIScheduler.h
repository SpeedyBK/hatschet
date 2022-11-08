//
// Created by nfiege on 13/11/19.
//

#ifndef HATSCHET_NONUNIFORMRATIONALIISCHEDULER_H
#define HATSCHET_NONUNIFORMRATIONALIISCHEDULER_H


#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT {
	class NonUniformRationalIIScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		/*!
		 * Constructor
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		NonUniformRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * print the rational II modulo schedule
		 */
		void printScheduleToConsole();
		/*!
		 * @brief print the MRTs of all resources after rational II scheduling and binding
		 */
		void printBindingToConsole();
      /*!
       * Function to set the solver Timeout
       * @param seconds
       */
      void setSolverTimeout(double timeoutInSeconds) override;

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
		 * container for ILP variables
		 */
		std::map<Vertex*,std::vector<ScaLP::Variable>> tVariables;
		/*!
		 * container for ILP variables
		 * map Vertex* -> b-Matrix
		 * first index of b-Matrix: sample index in {0, ..., #samples-1}
		 * second index of b-Matrix: modulo slot
		 */
		std::map<const Vertex*,std::vector<std::vector<ScaLP::Variable>>> bVariables;
	};
}




#endif //HATSCHET_NONUNIFORMRATIONALIISCHEDULER_H
