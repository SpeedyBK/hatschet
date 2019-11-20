//
// Created by nfiege on 12/11/19.
//

#ifndef HATSCHET_UNIFORMRATIONALIISCHEDULER_H
#define HATSCHET_UNIFORMRATIONALIISCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT {
	class UniformRationalIIScheduler : public RationalIISchedulerLayer, public ILPSchedulerBase {
	public:
		/*!
		 * Constructor
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		UniformRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * \brief getLifeTimes using the determined rational II
		 * lifetimes in rational II schedules are determined using the initiation intervall vector
		 * this is crucial because samples are inserted in not evenly spaced intervalls
		 * remark: overloaded function from the scheduler base class
		 * \return
		 */
		std::map<Edge*,vector<int> > getRatIILifeTimes();
		/*!
		 * dont use this function for rational II modulo schedules
		 * this function will throw an exception
		 * use getRatIILifeTimes()
		 * @return
		 */
		std::map<Edge*,int> getLifeTimes() override;
		/*!
		 * dont use this function fo rational II modulo schedules
		 * @return
		 */
		std::map<const Vertex*,int> getBindings() override;
		/*!
		 * generate a binding using the determined rational II schedule
		 * TODO figure out the best binding method (ILP?)
		 * @return
		 */
		vector<std::map<const Vertex*,int> > getRationalIIBindings() override;
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
		std::map<const Vertex*,std::vector<std::vector<ScaLP::Variable>>> bVariables;
	};
}


#endif //HATSCHET_UNIFORMRATIONALIISCHEDULER_H
