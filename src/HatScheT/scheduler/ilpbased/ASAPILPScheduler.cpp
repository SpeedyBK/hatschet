/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ASAPILPScheduler.h"
#include <cmath>
#include <utility>

namespace HatScheT {
    ASAPILPScheduler::ASAPILPScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
            : SchedulerBase(g, resourceModel), ILPSchedulerBase(std::move(solverWishlist)) {
        this->maxLatencyConstraint = -1;
    }

    void ASAPILPScheduler::schedule() {
        this->constructProblem();

        if (this->maxLatencyConstraint <= 0) throw HatScheT::Exception(
                    "ASAPILP Scheduler::schedule: irregular maxLatencyConstraint " +
                    to_string(this->maxLatencyConstraint));

//  this->begin = clock();
//  stat = this->solver->solve();
//  this->end = clock();
//  if(this->solvingTime == -1.0) this->solvingTime = 0.0;
//  this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

        startTimeTracking();
        stat = this->solver->solve();
        endTimeTracking();

        if (stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE ||
            stat == ScaLP::status::TIMEOUT_FEASIBLE)
            this->scheduleFound = true;
        if (stat == ScaLP::status::OPTIMAL) this->optimalResult = true;

        if (this->scheduleFound) {
            this->r = this->solver->getResult();
            this->fillSolutionStructure();
            this->II = this->getScheduleLength();
        } else {
            this->II = -1;
        }
    }

    void ASAPILPScheduler::constructProblem() {
        this->setVectorVariables();
        this->setGeneralConstraints();
        this->setResourceConstraints();
        this->setObjective();
    }

    void ASAPILPScheduler::setObjective() {
        //supersink latency objective
        ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink", 0, ScaLP::INF());
        for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
            Vertex *v = *it;
            unsigned int vTVecIndex = this->tIndices[v];

            this->solver->addConstraint(supersink - this->ti[vTVecIndex] >= 0);
        }
        this->solver->setObjective(ScaLP::minimize(supersink));
    }

    void ASAPILPScheduler::fillSolutionStructure() {
        for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
            Vertex *v = *it;
            unsigned int index = this->tIndices.at(v);
            ScaLP::Variable svTemp = this->ti[index];

            auto startTime = (int) round(this->r.values[svTemp]);
            this->startTimes.insert(make_pair(v, startTime));
        }
    }

    void ASAPILPScheduler::setGeneralConstraints() {
        for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
            Vertex *v = *it;
            unsigned int vTVecIndex = this->tIndices[v];

            vector<ScaLP::Variable> vBinaryVec;

            //every operation starts only once
            for (int i = 0; i <= this->maxLatencyConstraint; i++) {
                vBinaryVec.push_back(
                        ScaLP::newBinaryVariable("t_" + std::to_string(v->getId()) + "_" + std::to_string(i), 0, 1));
            }

            ScaLP::Term binStartTimeSum;
            ScaLP::Term binarySum;
            for (int i = 0; i < vBinaryVec.size(); i++) {
                binStartTimeSum = binStartTimeSum + i * vBinaryVec[i];
                binarySum = binarySum + vBinaryVec[i];
            }
            this->solver->addConstraint(binStartTimeSum - ti[vTVecIndex] == 0);
            this->solver->addConstraint(binarySum == 1);
            this->binVarMap.insert(make_pair(v, vBinaryVec));
        }

        for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
            Edge *e = *it;
            if (e->getDistance() > 0) continue;

            Vertex *src = &(e->getVertexSrc());
            unsigned int srcTVecIndex = this->tIndices[src];
            Vertex *dst = &(e->getVertexDst());
            unsigned int dstTVecIndex = this->tIndices[dst];

            this->solver->addConstraint(
                    ti[srcTVecIndex] - ti[dstTVecIndex] + this->resourceModel.getVertexLatency(src) + e->getDelay() <= 0);
        }
    }

    void ASAPILPScheduler::setVectorVariables() {
        for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
            Vertex *v = *it;
            int id = v->getId();

            this->ti.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id), 0, ScaLP::INF()));
            //store tIndex
            this->tIndices.insert(make_pair(v, ti.size() - 1));
        }
    }

    void ASAPILPScheduler::setResourceConstraints() {
        for (auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
            Resource *r = *it;
            if (this->resourceModel.getNumVerticesRegisteredToResource(r) == 0) continue;

            int ak = r->getLimit();
            if (ak == -1) continue;
            set<const Vertex *> verOfRes = this->resourceModel.getVerticesOfResource(r);


            for (int i = 0; i <= this->maxLatencyConstraint; i++) {
                ScaLP::Term binStartTimeVerticesSum;
                for (auto v: verOfRes) {
                    binStartTimeVerticesSum = binStartTimeVerticesSum + (this->binVarMap.at(v))[i];
                }

                this->solver->addConstraint(binStartTimeVerticesSum <= ak);
            }
        }
    }



}