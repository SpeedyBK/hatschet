//
// Created by nfiege on 12/20/22.
//

#ifndef HATSCHET_SATSCHEDULERBINENC_H
#define HATSCHET_SATSCHEDULERBINENC_H

#include <HatScheT/base/SATSchedulerBase.h>

namespace HatScheT {
	class SATSchedulerBinEnc : public SATSchedulerBase {
	public:
		SATSchedulerBinEnc(Graph& g, ResourceModel &resourceModel, int II=-1);

	protected:
		void scheduleIteration() override;

	private:
		enum trivialDecision {
			triviallySAT,
			triviallyUNSAT,
			noTrivialDecisionPossible
		};
		void defineLatLimits();
		trivialDecision setUpSolver();
		void resetContainer();
		void createLiterals();
		trivialDecision createClauses();
		void createIncrementalClauses();
		void fillSolutionStructure();

		bool inIncrementalMode();

		void createTimeSlotLimitationClauses();
		trivialDecision createDependencyClauses();
		void createResourceLimitationClauses();

		int create_arbitrary_clause(const std::vector<std::pair<int, bool>> &a);
		int create_arbitrary_clause(const std::vector<std::pair<int, bool>> &a, bool quiet);
		int create_full_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> c_i, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false}, int clauseMode=0);
		int create_half_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false}, int clauseMode=0);
		int create_not(int x, int not_x);
		int create_2x1_and(int a, int b, int y);
		int create_2x1_or(int a, int b, int y);
		int create_2x1_mux(int a, int b, int s, int y);
		int create_nand(int a, int b, int y);

		std::map<std::pair<Vertex*, int>, int> scheduleTimeVariables;
		std::map<Vertex*, int> scheduleTimeWordSize;
		std::map<std::pair<Edge*, int>, int> diffVariables;
		std::map<std::pair<Vertex*, int>, int> moduloSlotVariables;
		std::map<Vertex*, int> lastForbiddenTime;
		int constOneVar = -1;
		int constZeroVar = -1;

		int timeSlotLiteralCounter = 0;
		int resourceConstraintLiteralCounter = 0;
		int moduloSlotLiteralCounter = 0;
		int dependencyConstraintSubLiteralCounter = 0;
		int dependencyConstraintCompLiteralCounter = 0;

		int timeSlotConstraintClauseCounter = 0;
		int dependencyConstraintSubClauseCounter = 0;
		int dependencyConstraintCompClauseCounter = 0;
		int moduloConstraintClauseCounter = 0;
		int resourceConstraintClauseCounter = 0;
	};
}

#endif //HATSCHET_SATSCHEDULERBINENC_H
