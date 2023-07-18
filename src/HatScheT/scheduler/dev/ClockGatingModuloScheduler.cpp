//
// Created by nfiege on 7/14/23.
//

#include "ClockGatingModuloScheduler.h"
#include <sstream>
#include <cmath>

HatScheT::ClockGatingModuloScheduler::ClockGatingModuloScheduler(HatScheT::Graph &g,
																																 HatScheT::ResourceModel &resourceModel,
																																 const list<std::string> &solverWishlist, double II) :
	IterativeModuloSchedulerLayer(g, resourceModel, II), ILPSchedulerBase(solverWishlist)
{
	// nothing to do here
}

void HatScheT::ClockGatingModuloScheduler::scheduleIteration() {
	this->candidateII = static_cast<int>(this->II);
	this->constructProblem();
	auto stat = this->solver->solve();
	this->fillSolutionStructure(stat);
}

void HatScheT::ClockGatingModuloScheduler::scheduleInit() {
	for (auto &r : this->resourceModel.Resources()) {
		auto limit = r->getLimit();
		if (limit == UNLIMITED) {
			this->functionalUnits[r] = this->resourceModel.getNumVerticesRegisteredToResource(r);
		}
		else {
			this->functionalUnits[r] = limit;
		}
	}
}

void HatScheT::ClockGatingModuloScheduler::setUpSolverSettings() {
	this->solver->reset();
	this->solver->quiet   = this->solverQuiet;
	this->solver->timeout = static_cast<long>(this->solverTimeout);
	this->solver->threads = static_cast<int>(this->threads);
}

void HatScheT::ClockGatingModuloScheduler::constructDecisionVariables() {
	for (auto *v : this->g.Vertices()) {
		auto i = v->getId();
		this->t[i] = ScaLP::newIntegerVariable("t_"+std::to_string(i), 0.0, ScaLP::INF());
		this->y[i] = ScaLP::newIntegerVariable("y_"+std::to_string(i), 0.0, ScaLP::INF());
		this->n[i] = ScaLP::newIntegerVariable("n_"+std::to_string(i), 0.0, ScaLP::INF());
		for (int tau=0; tau<this->candidateII; tau++) {
			this->m[{i,tau}] = ScaLP::newBinaryVariable("m_"+std::to_string(i)+"_"+std::to_string(tau));
		}
		auto *r = this->resourceModel.getResource(v);
		auto fus = this->functionalUnits.at(r);
		for (int x=0; x<fus; x++) {
			this->b[{i,x}] = ScaLP::newBinaryVariable("b_"+std::to_string(i)+"_"+std::to_string(x));
		}
	}
	for (auto &r : this->resourceModel.Resources()) {
		auto verticesOfResource = this->resourceModel.getVerticesOfResource(r);
		for (auto &vi : verticesOfResource) {
			auto i = vi->getId();
			for (auto &vj : verticesOfResource) {
				auto j = vj->getId();
				if (i >= j) continue;
				this->sigma[{i,j}] = ScaLP::newBinaryVariable("sigma_"+std::to_string(i)+"_"+std::to_string(j));
				this->psi[{i, j}] = ScaLP::newBinaryVariable("phi_" + std::to_string(i) + "_" + std::to_string(j));
			}
		}
		auto fus = this->functionalUnits.at(r);
		auto maxDelay = this->maxClockGatingDelay[r];
		for (auto x=0; x<fus; x++) {
			for (auto c=0; c<clockDomainLimit; c++) {
				std::stringstream name;
				name << "f_" << r << "_" << x << "_" << c;
				this->f[{r,x,c}] = ScaLP::newBinaryVariable(name.str());
			}
			for (auto tau=0; tau<this->candidateII; tau++) {
				for (auto phi=0; phi<=maxDelay; phi++) {
					std::stringstream name;
					name << "alpha_" << r << "_" << x << "_" << tau << "_" << phi;
					this->alpha[{r,x,tau,phi}] = ScaLP::newBinaryVariable(name.str());
				}
				std::stringstream name;
				name << "h_" << r << "_" << x << "_" << tau;
				this->h[{r,x,tau}] = ScaLP::newBinaryVariable(name.str());
			}
		}
	}
	for (int c=0; c<this->clockDomainLimit; c++) {
		for (int tau=0; tau<this->candidateII; tau++) {
			this->z[{c,tau}] = ScaLP::newBinaryVariable("z_"+std::to_string(c)+"_"+std::to_string(tau));
		}
	}
}

void HatScheT::ClockGatingModuloScheduler::constructConstraints() {
	// T1
	for (auto &e : this->g.Edges()) {
		auto vi = &e->getVertexSrc();
		auto li = this->resourceModel.getVertexLatency(vi);
		auto i = vi->getId();
		auto j = e->getVertexDst().getId();
		auto delay = e->getDelay();
		auto prod = this->candidateII * e->getDistance();
		auto ti = this->t.at(i);
		auto tj = this->t.at(j);
		auto ni = this->n.at(i);
		this->solver->addConstraint(tj - ti - ni >= li + delay - prod);
	}
	// T2, T3, T4, R1
	for (auto v : this->g.Vertices()) {
		auto i = v->getId();
		auto *r = this->resourceModel.getResource(v);
		auto fus = this->functionalUnits.at(r);
		auto maxDelay = this->maxClockGatingDelay[r];
		auto ti = this->t.at(i);
		auto yi = this->y.at(i);
		auto ni = this->n.at(i);
		ScaLP::Term mSum;
		ScaLP::Term mSumWeighted;
		for (int tau=0; tau<this->candidateII; tau++) {
			auto mitau = this->m.at({i, tau});
			mSum += mitau;
			mSumWeighted += tau*mitau;
		}
		ScaLP::Term bSum;
		for (int x=0; x<fus; x++) {
			bSum += this->b.at({i,x});
		}
		// T4
		if (this->maxLatencyConstraint >= 0) {
			this->solver->addConstraint(ti + ni <= this->maxLatencyConstraint - this->resourceModel.getVertexLatency(v));
		}
		// T2
		this->solver->addConstraint(ti - this->candidateII*yi - mSumWeighted == 0.0);
		// T3
		this->solver->addConstraint(mSum == 1.0);
		// R1
		this->solver->addConstraint(bSum == 1.0);
	}
	// R2, R3, R4, C5
	for (auto *r : this->resourceModel.Resources()) {
		auto fus = this->functionalUnits.at(r);
		auto vertices = this->resourceModel.getVerticesOfResource(r);
		for (auto vi : vertices) {
			auto i = vi->getId();
			for (auto vj : vertices) {
				auto j = vj->getId();
				if (i >= j) continue;
				auto sigmaij = this->sigma.at({i,j});
				auto psiij = this->psi.at({i,j});
				// R4
				this->solver->addConstraint(sigmaij + psiij <= 1.0);
				// R2
				for (int tau=0; tau<this->candidateII; tau++) {
					auto mitau = this->m.at({i, tau});
					auto mjtau = this->m.at({j, tau});
					this->solver->addConstraint(mitau + mjtau - sigmaij <= 1.0);
				}
				// R3
				for (int x=0; x<fus; x++) {
					auto bix = this->b.at({i, x});
					auto bjx = this->b.at({j, x});
					this->solver->addConstraint(bix + bjx - psiij <= 1.0);
				}
			}
		}
		// C5
		for (auto x=0; x<fus; x++) {
			ScaLP::Term fsum;
			for (int c=0; c<this->clockDomainLimit; c++) {
				fsum += this->f.at({r,x,c});
			}
			this->solver->addConstraint(fsum == 1.0);
		}
	}
	// C1a, C1b, C2, C3, C4
	for (auto *r : this->resourceModel.Resources()) {
		auto fus = this->functionalUnits.at(r);
		auto maxDelay = this->maxClockGatingDelay[r];
		auto lat = r->getLatency();
		auto verticesOfResource = this->resourceModel.getVerticesOfResource(r);
		for (int x=0; x<fus; x++) {
			for (int tau=0; tau<this->candidateII; tau++) {
				auto hrxtau = this->h.at({r,x,tau});
				for (int phi=0; phi<=maxDelay; phi++) {
					ScaLP::Term hSum;
					for (int gamma=0; gamma<phi+lat; gamma++) {
						hSum += this->h.at({r, x, (tau+gamma)%this->candidateII});
					}
					// C3
					this->solver->addConstraint(-lat*this->alpha.at({r,x,tau,phi}) + hSum <= phi);
				}
				for (int c=0; c<this->clockDomainLimit; c++) {
					auto frxc = this->f.at({r,x,c});
					auto zctau = this->z.at({c,tau});
					// C1a, C1b
					this->solver->addConstraint(frxc + zctau - hrxtau <= 1.0);
					this->solver->addConstraint(frxc + hrxtau - zctau <= 1.0);
				}
				for (auto v : verticesOfResource) {
					auto i = v->getId();
					auto mitau = this->m.at({i,tau});
					auto bix = this->b.at({i,x});
					auto ni = this->n.at(i);
					// C2
					this->solver->addConstraint(hrxtau + mitau + bix <= 2.0);
					ScaLP::Term alphaSum;
					for (int phi=0; phi<=maxDelay; phi++) {
						alphaSum += this->alpha.at({r,x,tau,phi});
					}
					// C4
					this->solver->addConstraint(maxDelay * (2 - bix - mitau) + ni - alphaSum >= 0.0);
				}
			}
		}
	}
}

void HatScheT::ClockGatingModuloScheduler::setObjective() {
	ScaLP::Term obj;
	for (auto *r : this->resourceModel.Resources()) {
		auto eps = this->energyPerSample[r];
		for (int x=0; x<this->functionalUnits.at(r); x++) {
			for (int tau=0; tau<this->candidateII; tau++) {
				obj += eps * this->h.at({r,x,tau});
			}
		}
	}
	this->solver->setObjective(ScaLP::maximize(obj));
}

void HatScheT::ClockGatingModuloScheduler::resetContainer() {
	this->t.clear();
	this->y.clear();
	this->n.clear();
	this->m.clear();
	this->b.clear();
	this->sigma.clear();
	this->psi.clear();
	this->z.clear();
	this->f.clear();
	this->h.clear();
	this->alpha.clear();
}

void HatScheT::ClockGatingModuloScheduler::constructProblem() {
	this->setUpSolverSettings();
	this->resetContainer();
	this->constructDecisionVariables();
	this->constructConstraints();
	this->setObjective();
}

void HatScheT::ClockGatingModuloScheduler::fillSolutionStructure(ScaLP::status stat) {
	switch (stat) {
		case ScaLP::status::OPTIMAL: {
			this->scheduleFound = true;
			this->secondObjectiveOptimal = true;
			break;
		}
		case ScaLP::status::FEASIBLE:
		case ScaLP::status::TIMEOUT_FEASIBLE: {
			this->scheduleFound = true;
			// no change to first objective since we found a solution
			// second objective remains non-optimal
			break;
		}
		case ScaLP::status::INFEASIBLE_OR_UNBOUND:
		case ScaLP::status::INFEASIBLE: {
			this->scheduleFound = false;
			// no change to first objective since we successfully showed that there does not exist a solution
			// no info about second objective
			return;
		}
		default: {
			this->scheduleFound = false;
			// no idea about the solution space
			this->firstObjectiveOptimal = false;
			this->secondObjectiveOptimal = false;
			return;
		}
	}
	// let's retrieve the solution
	auto result = this->solver->getResult().values;
	std::cout << "#q# objective value = " << this->solver->getResult().objectiveValue << std::endl;
	for (auto &v : this->g.Vertices()) {
		auto i = v->getId();
		auto r = this->resourceModel.getResource(v);
		auto ti = this->t.at(i);
		auto maxDelay = this->maxClockGatingDelay.at(r);
		auto fus = this->functionalUnits.at(r);
		this->startTimes[v] = static_cast<int>(std::round(result.at(ti)));
		for (auto x=0; x<fus; x++) {
			auto bix = this->b.at({i,x});
			if (static_cast<bool>(std::round(result.at(bix)))) {
				this->binding[v] = x;
			}
		}
	}
	for (auto c=0; c<this->clockDomainLimit; c++) {
		this->clockOffTimes[c].resize(this->candidateII);
		for (auto tau=0; tau<this->candidateII; tau++) {
			auto zctau = this->z.at({c, tau});
			this->clockOffTimes[c][tau] = static_cast<bool>(std::round(result.at(zctau)));
		}
		for (auto &r : this->resourceModel.Resources()) {
			auto fus = this->functionalUnits.at(r);
			for (auto x=0; x<fus; x++) {
				auto frxc = this->f.at({r, x, c});
				if (static_cast<bool>(std::round(result.at(frxc)))) {
					this->clockDomainBinding[{r, x}] = c;
				}
			}
		}
	}
}

void HatScheT::ClockGatingModuloScheduler::setNumberOfClockDomains(const int &newNumClockDomains) {
	this->clockDomainLimit = newNumClockDomains;
}

void HatScheT::ClockGatingModuloScheduler::setEnergyPerSample(const HatScheT::Resource *r,
																															const double &newEnergyPerSample) {
	this->energyPerSample[r] = newEnergyPerSample;
}

void HatScheT::ClockGatingModuloScheduler::setMaxClockGatingDelay(const HatScheT::Resource *r,
																																	const int &newMaxClockGatingDelay) {
	this->maxClockGatingDelay[r] = newMaxClockGatingDelay;
}
