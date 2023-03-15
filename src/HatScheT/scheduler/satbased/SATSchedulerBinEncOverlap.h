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

		enum SATFormulationMode {
			implication, // compute m_i = t_i mod II using implications for all values that t_i can take (kind of inefficient but it works just fine)
			constMult // compute t_i = y_i * II + m_i using shift and add operations in SAT -> only works if SAT SCM lib is linked to HatScheT
		} satFormulationMode = implication;

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
		void createModuloComputationImplicationClauses();
		void createModuloComputationConstMultClauses();
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
		std::vector<int> create_unsigned_result_bitheap(const std::vector<std::tuple<std::vector<int>, bool, int>> &x, int* literalCounterPtr= nullptr, int* clauseCounterPtr= nullptr);

		std::map<std::pair<Vertex*, int>, int> scheduleTimeVariables;
		std::map<const Vertex*, int> scheduleTimeWordSize;
		std::map<const Vertex*, int> offsetIIWordSize;
		std::map<const Vertex*, int> offsetIIMultWordSize;
		std::map<std::pair<Edge*, int>, int> diffVariables;
		std::map<std::pair<const Vertex*, int>, int> offsetIIVariables;
		std::map<std::pair<const Vertex*, int>, int> offsetIIMultVariables;
		std::map<std::pair<const Vertex*, int>, int> moduloSlotVariables;
		std::map<std::pair<const Vertex*, int>, int> bindingVariables;
		std::map<std::tuple<const Vertex*, const Vertex*, int>, int> bindingOverlapVariables;
		std::map<std::tuple<const Vertex*, const Vertex*, int>, int> moduloOverlapVariables;
		std::map<const Vertex*, std::map<int, std::vector<int>>> adderGraphVariables;
		std::map<const Vertex*, int> lastForbiddenTime;
		int constOneVar = -1;
		int constZeroVar = -1;

		// used to compute t_i = II * y_i + m_i using shift and add operations
		void computeAdderGraph();
		std::unique_ptr<Graph> scmAdderGraph;
		std::vector<Vertex*> scmAdderGraphVertexOrder;
		int scmOutputShift = 0;
		int scmOutputConst = 0;

		// new
		int bindingLiteralCounter = 0;
		int overlapLiteralCounter = 0;
		int moduloComputationLiteralCounter = 0;

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
