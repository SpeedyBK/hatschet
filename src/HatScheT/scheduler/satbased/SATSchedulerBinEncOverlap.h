//
// Created by nfiege on 3/6/23.
//

#ifndef HATSCHET_SATSCHEDULERBINENCOVERLAP_H
#define HATSCHET_SATSCHEDULERBINENCOVERLAP_H


#include <HatScheT/base/SATSchedulerBase.h>

namespace HatScheT {
	class SATSchedulerBinEncOverlap : public SATSchedulerBase {
	public:
		SATSchedulerBinEncOverlap(Graph& g, ResourceModel &resourceModel, int II=-1);

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
		void createBindingLimitationClauses();
		trivialDecision createDependencyClauses();
		void createModuloComputationClauses();
		void createOverlapClauses();

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
		std::map<const Vertex*, int> scheduleTimeWordSize;
		std::map<std::pair<Edge*, int>, int> diffVariables;
		std::map<std::pair<const Vertex*, int>, int> moduloSlotVariables;
		std::map<std::pair<const Vertex*, int>, int> bindingVariables;
		std::map<std::tuple<const Vertex*, const Vertex*, int>, int> bindingOverlapVariables;
		std::map<std::tuple<const Vertex*, const Vertex*, int>, int> moduloOverlapVariables;
		std::map<const Vertex*, int> lastForbiddenTime;
		int constOneVar = -1;
		int constZeroVar = -1;

		// new
		int bindingLiteralCounter = 0;
		int overlapLiteralCounter = 0;

		int overlapClauseCounter = 0;
		int moduloComputationClauseCounter = 0;
		int bindingConstraintClauseCounter = 0;

		// old
		int timeSlotLiteralCounter = 0;
		int resourceConstraintLiteralCounter = 0;
		int moduloSlotLiteralCounter = 0;
		int dependencyConstraintSubLiteralCounter = 0;
		int dependencyConstraintCompLiteralCounter = 0;

		int timeSlotConstraintClauseCounter = 0;
		int dependencyConstraintSubClauseCounter = 0;
		int dependencyConstraintCompClauseCounter = 0;
		int resourceConstraintClauseCounter = 0;
	};
}


#endif //HATSCHET_SATSCHEDULERBINENCOVERLAP_H
