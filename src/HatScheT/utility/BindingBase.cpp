//
// Created by nfiege on 3/4/22.
//

#include "BindingBase.h"
#include "Exception.h"
#include <limits>

namespace HatScheT {

	BindingBase::BindingBase(HatScheT::Graph *g, HatScheT::ResourceModel *rm, std::map<Vertex*, int> sched, int II,
													std::set<const Resource*> commutativeOps) :
	g(g), rm(rm), sched(sched), II(II), timeBudget(300), quiet(true), solutionStatus("NOT_SOLVED"), wMux(1.0),
	wReg(1.0), maxMux(std::numeric_limits<double>::infinity()), commutativeOps(commutativeOps),
	maxReg(std::numeric_limits<double>::infinity()), obj(Binding::objective::minimize), numThreads(1)
	{
		// only init some members...
	}

	void BindingBase::bind() {
		throw HatScheT::Exception("BindingBase::bind: not implemented");
	}

	void BindingBase::getBinding(Binding::RegChainBindingContainer* b) {
		throw HatScheT::Exception("BindingBase::getBinding: not implemented");
	}

	void BindingBase::getBinding(Binding::BindingContainer* b) {
		throw HatScheT::Exception("BindingBase::getBinding: not implemented");
	}

	void BindingBase::setTimeout(unsigned int timeout) {
		this->timeBudget = timeout;
	}

	void BindingBase::setQuiet(bool q) {
		this->quiet = q;
	}

	void BindingBase::setMuxCostFactor(double wMux) {
		this->wMux = wMux;
	}

	void BindingBase::setRegCostFactor(double wReg) {
		this->wReg = wReg;
	}

	std::string BindingBase::getSolutionStatus() {
		return this->solutionStatus;
	}

	void BindingBase::setMuxLimit(double l) {
		if (l >= 0) {
			// valid value requested
			this->maxMux = l;
		}
		// invalid mux cost limit -> set to inifinity
		this->maxMux = std::numeric_limits<double>::infinity();
	}

	void BindingBase::setRegLimit(double l) {
		if (l >= 0) {
			// valid value requested
			this->maxReg = l;
		}
		// invalid reg cost limit -> set to inifinity
		this->maxReg = std::numeric_limits<double>::infinity();
	}

	void BindingBase::setObjective(Binding::objective o) {
		this->obj = o;
	}

	void BindingBase::setThreads(int t) {
		if (t <= 0) {
			throw HatScheT::Exception("Specified illegal number of threads: "+std::to_string(t)+" - only values >= 1 are allowed!");
		}
		this->numThreads = t;
	}
}