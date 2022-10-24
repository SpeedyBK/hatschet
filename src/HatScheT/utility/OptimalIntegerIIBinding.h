//
// Created by nfiege on 11/11/21.
//

#ifndef HATSCHET_OPTIMALINTEGERIIBINDING_H
#define HATSCHET_OPTIMALINTEGERIIBINDING_H

#ifdef USE_SCALP

#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <HatScheT/utility/Binding.h>
#include <HatScheT/utility/BindingBase.h>
#include <vector>
#include <map>
#include <list>

namespace HatScheT {

	class OptimalIntegerIIBinding : public BindingBase {
	public:
		/*!
		 * default constructor
		 * @param g input graph
		 * @param rm input resource model
		 * @param sched container with schedule time for each vertex in g
		 * @param II initiation interval
		 * @param portAssignments container that specifies input ports for
		 * dst vertices of each edge in g
		 * @param commutativeOps container with all commutative operation types
		 * @param sw ILP solver wishlist
		 */
		OptimalIntegerIIBinding(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments, std::set<const Resource*> commutativeOps = {}, std::list<std::string> sw = {});
		/*!
		 * override the function that actually does the binding
		 */
		void bind() override;
		/*!
		 * override base class's method
		 * @param b binding container to set
		 */
		void getBinding(Binding::RegChainBindingContainer* b) override;

	private:
		/*!
		 * ILP solver wishlist
		 */
		std::list<std::string> sw;
		/*!
		 * port assignment for each edge in g
		 */
		std::map<Edge*,int> portAssignments;
		/*!
		 * best solution found so far
		 */
		Binding::RegChainBindingContainer binding;
	};
}

#endif //USE_SCALP

#endif //HATSCHET_OPTIMALINTEGERIIBINDING_H
