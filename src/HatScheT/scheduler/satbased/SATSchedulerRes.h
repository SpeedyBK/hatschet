//
// Created by nfiege on 2/9/23.
//

#ifndef HATSCHET_SATSCHEDULERRES_H
#define HATSCHET_SATSCHEDULERRES_H

#include <HatScheT/base/SATSchedulerBase.h>

namespace HatScheT {
	class SATSchedulerRes : public SATSchedulerBase {
	public:
		SATSchedulerRes(Graph& g, ResourceModel &resourceModel, int II=-1);

	protected:
		void scheduleIteration() override;

	private:
		void defineLatLimits();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();

		void createScheduleTimeClauses();
		void createDependencyClauses();
		void createResourceLimitationClauses();

		int create_arbitrary_clause(const std::vector<std::pair<int, bool>> &a);
		int create_arbitrary_clause(const std::vector<std::pair<int, bool>> &a, bool quiet);
		int create_full_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> c_i, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false});
		int create_half_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false});
		int create_not(int x, int not_x);
		int create_2x1_and(int a, int b, int y);
		int create_2x1_or(int a, int b, int y);
		int create_2x1_mux(int a, int b, int s, int y);
		std::vector<int> create_bitheap(const std::vector<std::pair<std::vector<int>, bool>> &x);

		//std::map<std::pair<Vertex*, int>, int> moduloSlotVariables;
		std::map<std::pair<Vertex*, int>, int> scheduleTimeVariables;
		int constOneVar;
		int constZeroVar;

		//int moduloSlotLiteralCounter = 0;
		int scheduleTimeLiteralCounter = 0;
		int resourceConstraintLiteralCounter = 0;

		int scheduleTimeConstraintCounter = 0;
		int resourceConstraintClauseCounter = 0;
		int dependencyConstraintClauseCounter = 0;
	};
}

#endif //HATSCHET_SATSCHEDULERRES_H
