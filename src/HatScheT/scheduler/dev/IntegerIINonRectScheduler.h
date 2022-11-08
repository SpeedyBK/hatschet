//
// Created by nfiege on 7/8/21.
//

#ifndef HATSCHET_INTEGERIINONRECTSCHEDULER_H
#define HATSCHET_INTEGERIINONRECTSCHEDULER_H



#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

namespace HatScheT {
	class IntegerIINonRectScheduler :  public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

	public:
		/*!
		 * constructor
		 * @param g graph
		 * @param resourceModel
		 * @param solverWishlist
		 */
		IntegerIINonRectScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
		/*!
		 * \brief Attempts to schedule the given instances. The candidate II must be given in advance via setCandidateII
		 */
		virtual void schedule();
		/*!
		 * set candidate II for scheduling attempt
		 * this function must be called before schedule()
		 * @param II
		 */
		void setCandidateII(int II) {this->candII = II;}
		/*!
         * Function to set the solver Timeout
         * @param seconds
         */
        void setSolverTimeout(double timeoutInSeconds) override;

	protected:
		virtual void setUpSolverSettings();

		virtual void scheduleAttempt(int candII, bool &feasible, bool &proven);
		virtual void constructDecisionVariables(int candII);
		virtual void constructConstraints(int candII);
		virtual void setObjective();

		/*!
		 * not needed
		 */
		virtual void resetContainer(){/* unused */}
		/*!
		 * not needed
		 */
		virtual void constructProblem() {/* unused */}
	private:
		/*!
		 * candidate II
		 */
		int candII;
		/*!
		 * congruence class
		 */
		std::vector<std::map<const Vertex*, ScaLP::Variable>> b;
		/*!
		 * schedule time
		 */
		std::map<const Vertex*, ScaLP::Variable> t;
		/*!
		 * II multiplier with t = k * II + b
		 */
		std::map<const Vertex*, ScaLP::Variable> k;
	};
}

#endif //HATSCHET_INTEGERIINONRECTSCHEDULER_H
