//
// Created by nfiege on 4/13/22.
//

#ifndef HATSCHET_MINREGMULTISCHEDULER_H
#define HATSCHET_MINREGMULTISCHEDULER_H

#ifdef USE_SCALP

#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <map>
#include <vector>

namespace HatScheT {
	class MinRegMultiScheduler :  public IterativeModuloSchedulerLayer, public ILPSchedulerBase {
	public:
		MinRegMultiScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		void scheduleOLD();
		void disableMultipleStartTimes();
		void enableMultipleStartTimes();
		int getNumLifetimeRegs() const;
		bool validateScheduleAndBinding() const;
		void printScheduleAndBinding() const;
      /*!
       * Function to set the solver Timeout
       * @param seconds
       */
      void setSolverTimeout(double timeoutInSeconds) override;

	protected:
		void constructProblem() override;
		void setObjective() override;
	    void resetContainer() override;
	    /*!
         * \brief Schedule Iteration for one II.
         */
        void scheduleIteration() override;
        /*!
         * Initialize stuff before II-Search-Loop starts.
         */
        void scheduleInit() override;

	private:
		int candII;
		bool multipleStartTimesAllowed;
		int numLifetimeRegs;
		void fillSolutionStructure();
		double bigM;
		std::map<Vertex*, std::vector<int>> startTimes;
		std::map<Vertex*, std::vector<int>> bindings;
		void initScheduler();
		std::map<Vertex*,std::set<Edge*>> incomingEdges;
		std::map<Vertex*,std::set<Edge*>> outgoingEdges;
		/*!
		 * must be called after filling incoming/outgoing edges
		 * @param v vertex
		 * @return maximum number of start times assignments that this vertex can have
		 */
		int computeNumberOfPossibleStartTimes(Vertex* v);
		std::map<Vertex*, int> numberOfPossibleStartTimes;
		void setUpSolver();
		void createScaLPVariables();
		std::map<Vertex*, std::vector<ScaLP::Variable>> t;
		std::map<Vertex*, std::vector<ScaLP::Variable>> z;
		std::map<std::pair<Vertex*,int>, std::vector<ScaLP::Variable>> b;
		std::map<std::pair<Vertex*,int>, std::vector<ScaLP::Variable>> bHat;
		std::map<Edge*, std::vector<ScaLP::Variable>> n;
		std::map<std::pair<Resource*, int>, ScaLP::Variable> xi;
		std::map<std::pair<Vertex*, int>, std::vector<ScaLP::Variable>> rho;
		std::map<std::pair<Vertex*, int>, std::vector<ScaLP::Variable>> rhoHat;
		std::map<Edge*, std::vector<ScaLP::Variable>> s;
		unsigned int ilpConstraintCounter;
		unsigned int ilpVariableCounter;
	};
}

#endif //USE_SCALP
#endif //HATSCHET_MINREGMULTISCHEDULER_H
