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
	class NonUniformRationalIIScheduler : public SchedulerBase, public ILPSchedulerBase, public RationalIISchedulerLayer, public IterativeSchedulerBase {
	public:
		/*!
		 * Constructor
		 * @param g
		 * @param resourceModel
		 * @param solverWishlist
		 */
		NonUniformRationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * the main function of the scheduler. The rational II scheduler tries to identify high throughput schedules on
		 * the theoretical min II boundary. For this the variables s / m are used
		 */
		void schedule() override;
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
		/*!
		 * @brief the edgePortMapping can be used to optmize the binding in order
		 * to minimize the effort for MUX hardware
		 * @param epm
		 */
		void setedgePortMapping(map<Edge*, pair<int, int> > epm){
			this->edgePortMapping = epm;
		}
		map<Edge*, pair<int, int> > getedgePortMapping(){
			return this->edgePortMapping;
		}
		/*!
		 * @brief iteration start of s
		 * @return
		 */
		int getS_Start(){return this->s_start;}
		/*!
		 * @brief iteration start of m
		 * @return
		 */
		int getM_Start(){return this->m_start;}
		/*!
		 * @brief found value for s (-1 if no schedule was found)
		 * @return
		 */
		int getS_Found(){return this->s_found;}
		/*!
		 * @brief found value for m (-1 if no schedule was found)
		 * @return
		 */
		int getM_Found(){return this->m_found;}
	private:
		/*!
		 * verify the found schedule (stored in startTimesVector)
		 * 	1) based on rational II verifier
		 * 	2) based on integer II verifier of unrolled graph
		 * throw error if the verifiers lead to different results!
		 * @return if the schedule is valid
		 */
		bool verifySchedule();
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
		 * this function sets the s and m values in a way that not needed values are skipped
		 * and the rational II becomes as small as possible
		 */
		void autoSetMAndS();
		/*!
		 * dito
		 */
		void autoSetNextMAndS();
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
		/*!
		 * the minimum interger II that is possible
		 */
		int integerMinII;
		/*!
		 * buffer
		 */
		double tpBuffer;
		/*!
		 * flag
		 */
		bool minRatIIFound;
		/*!
		 * @brief the edgePortMapping can be used to optmize the binding in order
		 * to minimize the effort for MUX hardware
		 */
		map<HatScheT::Edge*, pair<int,int> > edgePortMapping;
		/*!
		 * @brief the s value for the iteration start
		 */
		int s_start;
		/*!
		 * @brief the m value for the iteration start
		 */
		int m_start;
		/*!
		 * @brief the identified s value
		 */
		int s_found;
		/*!
		 * @brief the identified s value
		 */
		int m_found;
	};
}




#endif //HATSCHET_NONUNIFORMRATIONALIISCHEDULER_H
