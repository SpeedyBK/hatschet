#include <HatScheT/utility/SDCSolverBellmanFord.h>
#include "HatScheT/scheduler/dev/DeSouzaRosa23NIS.h"

    void HatScheT::NonIterativeModuloScheduler::scheduleIteration()
    {
        firstObjectiveOptimal = this->firstRun;
        int curII = (int)this->II;
        std::cout << "currentII: " << curII << std::endl;
        std::vector<Vertex*> orderedInstructions;
        if (custom)
        {
            std::vector<Vertex*> cycleInstructions;
            // calc actual slacks from path lengths
            for (auto& cycle : this->cyclePaths)
            {
                int slack = cycle.first->getDistance() * curII - cycle.second.length;
                for (auto inst : cycle.second)
                {
                    if (this->wMap.at(inst).weight == -1)
                    {
                        this->wMap.at(inst).weight = slack;
                        cycleInstructions.push_back(inst);
                    }
                    else if (slack < this->wMap.at(inst).weight)
                    {
                        this->wMap.at(inst).weight = slack;
                    }
                }
            }

            // order cycle instructions
            std::sort(cycleInstructions.begin(), cycleInstructions.end());

            // establish topological order
            //std::cout << "cycle insts" << std::endl;
            for (auto inst : cycleInstructions)
            {
                //std::cout << inst->getName() << std::endl;
                std::vector<Vertex*> toAdd = establishTopologicalOrderForCycle(inst, this->wMap);
                orderedInstructions.insert(orderedInstructions.end(), toAdd.begin(), toAdd.end());
                //std::cout << "ordered cycles" << std::endl;
                //for (auto tmp : orderedInstructions)
                //{
                //    std::cout << tmp->getName() << " ";
                //}
                //std::cout << std::endl;
            }
            //reset for next ii
            for (auto inst : cycleInstructions)
            {
                this->wMap.at(inst).weight = -1;
                this->wMap.at(inst).ordered = false;
            }
            // merge cycle insts and forward insts to orderedInstructions
            orderedInstructions.insert(orderedInstructions.end(), orderedForwardInstructions.begin(), orderedForwardInstructions.end());
        }
        else
        {
            std::map<Vertex*, bool> vMap;
            wMap.clear();
            //instructions that are part of a cycle
            std::vector<Vertex*> instructions;
            {
                std::vector<CycleVec> cycles;
                {
                    std::vector<Edge*> backEdges;
                    {
                        for (auto edge : this->g.Edges())
                        {
                            if (edge->getDistance() > 0)
                            {
                                //std::cout << "for edge:" << edge->getVertexSrc().getName() << "->" << edge->getVertexDst().getName() << " with distance" << edge->getDistance() << std::endl;
                                backEdges.push_back(edge);
                            }
                        }

                        for (auto backEdge : backEdges)
                        {
                            vector<CycleVec> backEdgeCycles = getCycleAndSlacks(&backEdge->getVertexDst(), &backEdge->getVertexSrc(), 0, backEdge->getDistance() * curII, vMap);
                            cycles.insert(cycles.end(), backEdgeCycles.begin(), backEdgeCycles.end());
                        }
                    }

                    std::sort(cycles.begin(), cycles.end());

                    for (auto& cycle : cycles)
                    {
                        for (auto inst : cycle)
                        {
                            if (this->wMap.insert({ inst, OrderedWeight(cycle.slack, false) }).second)
                            {
                                instructions.push_back(inst);
                            }
                        }
                    }
                }

                //std::cout << "cycle insts" << std::endl;
                for (auto inst : instructions)
                {
                    //std::cout << inst->getName() << std::endl;
                    std::vector<Vertex*> toAdd = establishTopologicalOrderForCycle(inst, this->wMap);
                    orderedInstructions.insert(orderedInstructions.end(), toAdd.begin(), toAdd.end());
                    //std::cout << "ordered cycles" << std::endl;
                    //for (auto tmp : orderedInstructions)
                    //{
                    //    std::cout << tmp->getName() << " ";
                    //}
                    //std::cout << std::endl;
                }
            }

            //std::cout << "cycles done" << std::endl;
            //instructions that are part of no cycle
            std::vector<PathVec> paths;
            for (auto inst : this->g.Vertices())
            {
                if (this->wMap.insert({ inst, OrderedWeight(1, false) }).second)
                {
                    if (this->g.isSinkVertex(inst))
                    {
                        std::vector<PathVec> toAdd = getBackwardsPathsAndLengths(inst, 0, vMap);
                        paths.insert(paths.end(), toAdd.begin(), toAdd.end());
                    }
                    else if (this->g.isSourceVertex(inst))
                    {
                        std::vector<PathVec> toAdd = getForwardsPathsAndLengths(inst, 0, vMap);
                        paths.insert(paths.end(), toAdd.begin(), toAdd.end());
                    }
                }
            }

        std::sort(paths.begin(), paths.end());

        std::vector<Vertex*> instructions2;
        for (auto& path : paths)
        {
            //std::cout << "pathLength: " << path.length << std::endl;
            for (auto inst : path)
            {
                //std::cout <<"inst: " << inst->getName() << std::endl;
                if (!this->wMap.at(inst).ordered && this->wMap.at(inst).weight == 1)
                {
                    this->wMap.at(inst).weight = -path.length;
                    instructions2.push_back(inst);
                }
            }
        }

        std::vector<Edge*> edges;
        for (auto inst : instructions2)
        {
            std::set<Edge*> tmpEdges = this->g.getOutgoingEdges(inst);
            edges.insert(edges.end(), tmpEdges.begin(), tmpEdges.end());
        }

        //std::sort(edges.begin(), edges.end(), [&wMap](Edge* a, Edge* b)
        //	{
        //		return wMap.at(&a->getVertexDst()).weight < wMap.at(&b->getVertexDst()).weight;
        //	});

        vector<Vertex*> orderedInstructions2;

        for (auto edge : edges)
        {
            auto isSameVertex1 = [&edge](Vertex* v) { return edge->getVertexSrc().getId() == v->getId(); }; //TODO a bit ugly?
            if (std::find_if(orderedInstructions2.begin(), orderedInstructions2.end(), isSameVertex1) == orderedInstructions2.end())
            {
                orderedInstructions2.push_back(&edge->getVertexSrc());
            }
            auto isSameVertex2 = [&edge](Vertex* v) { return edge->getVertexDst().getId() == v->getId(); }; //TODO a bit ugly?
            if (std::find_if(orderedInstructions2.begin(), orderedInstructions2.end(), isSameVertex2) == orderedInstructions2.end())
            {
                orderedInstructions2.push_back(&edge->getVertexDst());
            }
        }

        for (auto inst : orderedInstructions2)
        {
            //std::cout << inst->getName() << std::endl;
            std::vector<Vertex*> toAdd = establishTopologicalOrderForPath(inst, this->wMap);
            orderedInstructions.insert(orderedInstructions.end(), toAdd.begin(), toAdd.end());
        }

        //std::cout << "paths done" << std::endl;
    }
        
        //MRT
        std::map<int, int> asapStartTimes = getASAPNonRestrictedStartTimes(curII);

        std::map<Vertex*, int> increments;
        std::map<Vertex*, int> congruencies;

        //std::cout << "ordered" << std::endl;
        for (auto inst : orderedInstructions)
		{
            auto asapKeyVal = asapStartTimes.find(inst->getId());
            int asapStartTime = asapKeyVal == asapStartTimes.end() ? 0 : asapKeyVal->second;
            //std::cout << inst->getName() << " with weight: " << this->wMap.at(inst).weight << " scheduled at: " << asapStartTime << std::endl;
            int increment = increments.count(inst) > 0 ? increments.at(inst) : 0;
            int m0 = (asapStartTime % curII + increment) % curII;
            if (this->resourceModel.getResource(inst)->isUnlimited())
			{
                congruencies.insert({ inst, m0 });
            }
            else
			{
                int m = m0;
                int inc = 0;
                while (congruencies.count(inst) == 0)
				{
                    if (Utility::resourceAvailable(congruencies, &this->resourceModel, this->resourceModel.getResource(inst), inst, m, curII))
					{
                        congruencies.insert({ inst, m });
                    }
                    else
					{
                        m++;
                        inc++;
                    }
                    propagateIncrement(increments, inst, inc);
                }
            }
        }

        SDCSolverBellmanFord solver;
        //solver.setQuiet(false);
        for (auto edge : this->g.Edges())
		{
            double val1 = static_cast<double>(-(getLatency(&edge->getVertexSrc()) + edge->getDelay()) + edge->getDistance() * curII - (congruencies.at(&edge->getVertexSrc()) - congruencies.at(&edge->getVertexDst())));
            int val2 = (int)std::floor(val1 / static_cast<double>(curII));
            solver.addBaseConstraint(edge->getVertexSrc().getId(), edge->getVertexDst().getId(), val2);
            //std::cout << "Added SDC constraint 't_" << edge->getVertexSrcName() << " - t_" << edge->getVertexDstName() << " <= " << val2 << "' (m_i - m_j = " << congruencies.at(&edge->getVertexSrc()) << " - " << congruencies.at(&edge->getVertexDst()) << " = " << congruencies.at(&edge->getVertexSrc()) - congruencies.at(&edge->getVertexDst()) << ")" << std::endl;
        }

        solver.solve();
        if (!solver.getSolutionFound())
        {
            //std::cout << "could not find solution" << std::endl;
            this->scheduleFound = false;
            return;
        }

        std::map<int, int> helpers = solver.getSolution();
        int smallest = helpers.begin()->second;
        for (auto& helper = ++helpers.begin(); helper != helpers.end(); helper++)
        {
            if (helper->second < smallest)
            {
                smallest = helper->second;
            }
        }

		for (auto& helper : helpers)
		{
		    //std::cout << "scheduling node " << &this->g.getVertexById(helper.first).getName() << " at time " << (helper.second + offset) * curII + congruencies.at(&this->g.getVertexById(helper.first)) << std::endl;
			this->startTimes.insert({ &this->g.getVertexById(helper.first), (helper.second - smallest) * curII + congruencies.at(&this->g.getVertexById(helper.first)) });
		}

		//instructions without edges
		for (auto inst : this->g.Vertices())
		{
			if (this->startTimes.count(inst) == 0)
			{
				for (int i = congruencies.at(inst); i < curII; i++)
				{
				    if (this->startTimes.count(inst) == 0)
                    {
                        if (Utility::resourceAvailable(this->startTimes, &this->resourceModel, this->resourceModel.getResource(inst), inst, i))
                        {
                            this->startTimes.insert({ inst, i });
                        }
                    }
				}
			}
			//std::cout << "t_" << inst->getName() << " = " << this->startTimes.at(inst) << std::endl;
		}
		this->scheduleFound = true;
		this->firstRun = false;
	}

	void HatScheT::NonIterativeModuloScheduler::propagateIncrement(std::map<Vertex*, int>& increments, Vertex* currentInst, int increment)
	{
        if (!increments.insert({ currentInst, increment }).second)
		{
            increments.at(currentInst) += increment;
        }
        for (auto inst : this->g.getSuccessors(currentInst))
		{
            if (isSinkForCurrentLoop(*inst))
            {
                propagateIncrement(increments, inst, increment);
            }
        }
    }

	bool HatScheT::NonIterativeModuloScheduler::isSourceForCurrentLoop(Vertex& instruction)
	{
		if (!this->g.isSourceVertex(&instruction))
		{
			for (auto edge : this->g.getIncomingEdges(&instruction))
			{
				if (edge->getDistance() == 0)
				{
					return false;
				}
			}
		}
		return true;
	}

	bool HatScheT::NonIterativeModuloScheduler::isSinkForCurrentLoop(Vertex& instruction)
	{
		if (!this->g.isSinkVertex(&instruction))
		{
			for (auto edge : this->g.getOutgoingEdges(&instruction))
			{
				if (edge->getDistance() == 0)
				{
					return false;
				}
			}
		}
		return true;
	}

	std::vector<HatScheT::NonIterativeModuloScheduler::CycleVec> HatScheT::NonIterativeModuloScheduler::getCycleAndSlacks(Vertex* currentInst, Vertex* backEdgeInst, int partialSlack, int backEdgeDistance, std::map<Vertex*, bool>& vMap)
	{
		std::vector<CycleVec> returnCycles;
		if (this->pathBools.count(backEdgeInst) != 0 && this->pathBools.at(backEdgeInst).count(currentInst) != 0)
		{
			return returnCycles;
		}

		//TODO: should this actually be an extra case (single nodes, with backedge)?
        //if (this->g.isSourceVertex(backEdgeInst))
        //{
        //    CycleVec cycle;
        //    cycle.push_back(backEdgeInst);
        //    cycle.slack = backEdgeDistance - getLatency(backEdgeInst);
        //    returnCycles.push_back(cycle);
        //    return returnCycles;
        //}

		for (auto edge : this->g.getOutgoingEdges(currentInst))
		{
			if (edge->getDistance() > 0)
			{
				continue;
			}

			if (edge->getVertexDst().getId() == backEdgeInst->getId())
			{
                if (vMap.count(currentInst) == 0 || !vMap.at(currentInst))
                {
				    CycleVec cycle;
                    cycle.slack = backEdgeDistance - (getLatency(currentInst) + partialSlack);

                    if (vMap.count(&edge->getVertexDst()) == 0 || !vMap.at(&edge->getVertexDst()))
                    {
                        cycle.push_back(&edge->getVertexDst());
                        cycle.slack -= getLatency(&edge->getVertexDst()) + edge->getDelay();
                    }

                    cycle.push_back(currentInst);
                    returnCycles.push_back(cycle);
                }
			}
			else
			{
                bool inserted = vMap.insert({ currentInst, true }).second;
                std::vector<CycleVec> cycles;
                if (inserted || !vMap.at(currentInst))
                {
                    if (!inserted)
                    {
                        vMap.at(currentInst) = true;
                    }
                    cycles = getCycleAndSlacks(&edge->getVertexDst(), backEdgeInst, partialSlack + getLatency(currentInst) + edge->getDelay(), backEdgeDistance, vMap);
                    vMap.at(currentInst) = false;
                }
                else
                {
                    return returnCycles;
                }

                for (auto& cycle : cycles)
                {
                    cycle.push_back(currentInst);
                }
                returnCycles.insert(returnCycles.end(), cycles.begin(), cycles.end());
			}
		}

		if (returnCycles.empty())
		{
		    if (this->pathBools.count(backEdgeInst) == 0)
            {
                this->pathBools.insert({backEdgeInst, {{currentInst, true}}});
            }
		    else
            {
                this->pathBools.at(backEdgeInst).insert({currentInst, true});
            }
		}

		return returnCycles;
	}

	std::vector<HatScheT::NonIterativeModuloScheduler::PathVec> HatScheT::NonIterativeModuloScheduler::getBackwardsPathsAndLengths(Vertex* currentInst, int partialLength, std::map<Vertex*, bool>& vMap)
	{
		std::vector<PathVec> paths;

        if (this->g.isSourceVertex(currentInst))
        {
            PathVec tmpPath;
            tmpPath.push_back(currentInst);
            tmpPath.length = getLatency(currentInst);
            paths.push_back(tmpPath);
            return paths;
        }

		for (auto edge : this->g.getIncomingEdges(currentInst))
		{
			if (edge->getDistance() > 0)
			{
				continue;
			}

			if (isSourceForCurrentLoop(edge->getVertexSrc()))
			{
                if (vMap.count(currentInst) == 0 || !vMap.at(currentInst))
                {
                    PathVec tmpPath;
                    tmpPath.length = partialLength + getLatency(currentInst);

                    if (vMap.count(&edge->getVertexSrc()) == 0 || !vMap.at(&edge->getVertexSrc()))
                    {
                        tmpPath.push_back(&edge->getVertexSrc());
                        tmpPath.length += getLatency(&edge->getVertexSrc()) + edge->getDelay();
                    }

                    tmpPath.push_back(currentInst);
                    paths.push_back(tmpPath);
                }
			}
			else
			{
                bool inserted = vMap.insert({ currentInst, true }).second;
                std::vector<PathVec> tmpPaths;
                if (inserted || !vMap.at(currentInst))
                {
                    if (!inserted)
                    {
                        vMap.at(currentInst) = true;
                    }
                    tmpPaths = getBackwardsPathsAndLengths(&edge->getVertexSrc(), partialLength + getLatency(currentInst) + edge->getDelay(), vMap);
                    vMap.at(currentInst) = false;
                }

                for (auto& path : tmpPaths)
                {
                    path.push_back(currentInst);
                }
                paths.insert(paths.end(), tmpPaths.begin(), tmpPaths.end());
            }
		}
		return paths;
	}

    std::vector<HatScheT::NonIterativeModuloScheduler::PathVec> HatScheT::NonIterativeModuloScheduler::getForwardsPathsAndLengths(Vertex* currentInst, int partialLength, std::map<Vertex*, bool>& vMap)
    {
        std::vector<PathVec> paths;

        if (this->g.isSinkVertex(currentInst))
        {
            PathVec tmpPath;
            tmpPath.push_back(currentInst);
            tmpPath.length = getLatency(currentInst);
            paths.push_back(tmpPath);
            return paths;
        }

        for (auto edge : this->g.getOutgoingEdges(currentInst))
        {
            if (edge->getDistance() > 0)
            {
                continue;
            }

            if (isSinkForCurrentLoop(edge->getVertexDst()))
            {
                if (vMap.count(currentInst) == 0 || !vMap.at(currentInst))
                {
                    PathVec tmpPath;
                    tmpPath.length = partialLength + getLatency(currentInst);

                    if (vMap.count(&edge->getVertexDst()) == 0 || !vMap.at(&edge->getVertexDst()))
                    {
                        tmpPath.push_back(&edge->getVertexDst());
                        tmpPath.length += getLatency(&edge->getVertexDst()) + edge->getDelay();
                    }

                    tmpPath.push_back(currentInst);
                    paths.push_back(tmpPath);
                }
            }
            else
            {
                bool inserted = vMap.insert({ currentInst, true }).second;
                std::vector<PathVec> tmpPaths;
                if (inserted || !vMap.at(currentInst))
                {
                    if (!inserted)
                    {
                        vMap.at(currentInst) = true;
                    }
                    tmpPaths = getForwardsPathsAndLengths(&edge->getVertexDst(), partialLength + getLatency(currentInst) + edge->getDelay(), vMap);
                    vMap.at(currentInst) = false;
                    for (auto& path : tmpPaths)
                    {
                        path.push_back(currentInst);
                    }
                    paths.insert(paths.end(), tmpPaths.begin(), tmpPaths.end());
                }
            }
        }
        return paths;
    }

	std::vector<HatScheT::Vertex*> HatScheT::NonIterativeModuloScheduler::establishTopologicalOrderForCycle(Vertex* currentInst, std::map<Vertex*, OrderedWeight>& wMap)
	{
		std::vector<Vertex*> orderedInstructions;
		if (wMap.at(currentInst).ordered)
		{
            return orderedInstructions;
        }

        std::vector<Vertex*> toAdd;

        for (auto edge : this->g.getIncomingEdges(currentInst))
        {
            if (edge->getDistance() > 0)
            {
                continue;
            }

            if (wMap.count(&edge->getVertexSrc()) != 0 && !wMap.at(&edge->getVertexSrc()).ordered)
            {
                toAdd.push_back(&edge->getVertexSrc());
            }
        }
        std::sort(toAdd.begin(), toAdd.end(), [&wMap](Vertex* a, Vertex* b)
		{
		    return wMap.at(a).weight < wMap.at(b).weight;
		});

        for (auto toAddInstruction : toAdd)
		{
            std::vector<Vertex*> toAdd2 = establishTopologicalOrderForCycle(toAddInstruction, wMap);
            orderedInstructions.insert(orderedInstructions.end(), toAdd2.begin(), toAdd2.end());
        }

        orderedInstructions.push_back(currentInst);
        wMap.at(currentInst).ordered = true;

        return orderedInstructions;
	}

    std::vector<HatScheT::Vertex*> HatScheT::NonIterativeModuloScheduler::establishTopologicalOrderForPath(Vertex* currentInst, std::map<Vertex*, OrderedWeight>& wMap)
    {
        std::vector<std::vector<Vertex *>> predPaths;

        for (auto edge : this->g.getIncomingEdges(currentInst))
        {
            if (edge->getDistance() > 0)
            {
                continue;
            }

            std::vector<Vertex *> toAdd = establishTopologicalOrderForPath(&edge->getVertexSrc(), wMap);
            if (!toAdd.empty())
            {
                predPaths.push_back(toAdd);
            }
        }

        //std::cout << "Node: " << currentInst->getName() << std::endl;
        //for (const auto& vec : predPaths)
        //{
        //    std::cout << " [ ";
        //    for (auto inst : vec)
        //    {
        //        std::cout << inst->getName() << " , ";
        //    }
        //    std::cout << " ] " << std::endl;
        //}

        std::vector<Vertex *> orderedInstructions;

        std::vector<int> indices(predPaths.size(), 0);
        int currentWeight;

        auto findHeaviestNode = [&predPaths, &indices, &wMap]()
                {
                    //std::cout << " Indices ";
                    //std::cout << " [ ";
                    //for (auto index : indices)
                    //{
                    //    std::cout << index << " , ";
                    //}
                    //std::cout << " ] " << std::endl;
                    int smallest;
                    for (int i = 0; i < indices.size(); i++)
                    {
                        if (indices.at(i) < predPaths.at(i).size())
                        {
                            smallest = wMap.at(predPaths.at(i).at(indices.at(i))).weight;
                            break;
                        }
                    }
                    for (int i = 0; i < indices.size(); i++)
                    {
                        if (indices.at(i) < predPaths.at(i).size())
                        {
                            int candidate = wMap.at(predPaths.at(i).at(indices.at(i))).weight;
                            if (candidate < smallest)
                            {
                                smallest = candidate;
                            }
                        }
                    }
                    return smallest;
                };

        bool continueLoop = true;
        while (continueLoop)
        {
            currentWeight = findHeaviestNode();
            //std::cout << "heaviest: " << currentWeight << std::endl;
            for (int i = 0; i < predPaths.size(); i++)
            {
                int j = indices.at(i);
                for (; j < predPaths.at(i).size(); j++)
                {
                    if (wMap.at(predPaths.at(i).at(j)).weight == currentWeight)
                    {
                        orderedInstructions.push_back(predPaths.at(i).at(j));
                        indices.at(i) = j + 1;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            //std::cout << " Indices ";
            //std::cout << " [ ";
            //for (auto index : indices)
            //{
            //    std::cout << index << " , ";
            //}
            //std::cout << " ] " << std::endl;

            continueLoop = false;
            for (int i = 0; i < predPaths.size(); i++)
            {
                if (indices.at(i) < predPaths.at(i).size())
                {
                    continueLoop = true;
                    break;
                }
            }
        }

        if (wMap.count(currentInst) != 0 && !wMap.at(currentInst).ordered)
        {
            orderedInstructions.push_back(currentInst);
            wMap.at(currentInst).ordered = true;
        }

        //std::cout << "Returning from Node: " << currentInst->getName() << std::endl;
        return orderedInstructions;
    }

	bool HatScheT::NonIterativeModuloScheduler::CycleVec::operator < (const CycleVec& vec) const
	{
		return (slack < vec.slack);
	}
	
	bool HatScheT::NonIterativeModuloScheduler::PathVec::operator < (const PathVec& vec) const
	{
		return (length > vec.length);
	}

	HatScheT::NonIterativeModuloScheduler::OrderedWeight::OrderedWeight(int w, bool o) : weight(w), ordered(o) { };

	std::map<int, int> HatScheT::NonIterativeModuloScheduler::getASAPNonRestrictedStartTimes(int curII)
	{
		std::map<int, int> idTimeMap;
        SDCSolverBellmanFord solver;
        solver.setQuiet(false);
        //std::cout << "currentII: " << curII << std::endl;
        for (auto edge : this->g.Edges())
        {
            //double val1 = static_cast<double>(-(getLatency(&edge->getVertexSrc()) + edge->getDelay()) + edge->getDistance() * curII - (congruencies.at(&edge->getVertexSrc()) - congruencies.at(&edge->getVertexDst())));
            //int val2 = (int)std::floor(val1 / static_cast<double>(curII));
            int val = edge->getDistance() * curII - getLatency(&edge->getVertexSrc()) - edge->getDelay();
            solver.addBaseConstraint(edge->getVertexSrc().getId(), edge->getVertexDst().getId(), val);
            //std::cout << "Added SDC constraint 't_" << edge->getVertexSrcName() << " - t_" << edge->getVertexDstName() << " <= " << val2 << "' (m_i - m_j = " << congruencies.at(&edge->getVertexSrc()) << " - " << congruencies.at(&edge->getVertexDst()) << " = " << congruencies.at(&edge->getVertexSrc()) - congruencies.at(&edge->getVertexDst()) << ")" << std::endl;
        }

        solver.solve();
        if (!solver.getSolutionFound())
        {
            std::cout << "could not find asap schedule" << std::endl;
            //TODO correct exception
        }
        else
        {
            std::map<int, int> helpers = solver.getSolution();
            int smallest = helpers.begin()->second;
            for (auto& helper = ++helpers.begin(); helper != helpers.end(); helper++)
            {
                if (helper->second < smallest)
                {
                    smallest = helper->second;
                }
            }

            idTimeMap = solver.getSolution();
            for (auto& keyVal : idTimeMap)
            {
                keyVal.second -= smallest;
            }
        }
		return idTimeMap;
	}

    std::vector<HatScheT::NonIterativeModuloScheduler::PathVec> HatScheT::NonIterativeModuloScheduler::getPathsAndCycles(Vertex* currentInst, int currentLength, std::map<Edge*, std::vector<PathVec>>& currentCyclePaths, std::map<Vertex*, bool>& vMap)
    {
        std::vector<PathVec> paths;
        if (this->g.isSourceVertex(currentInst))
        {
            if (this->g.isSinkVertex(currentInst))
            {
                PathVec tmpPath;
                tmpPath.push_back(currentInst);
                tmpPath.length = getLatency(currentInst);
                paths.push_back(tmpPath);
                return paths;
            }
            else if (isSinkForCurrentLoop(*currentInst))
            {
                for (auto edge : this->g.getOutgoingEdges(currentInst))
                {
                    PathVec cycle;
                    cycle.push_back(currentInst);
                    cycle.length = getLatency(currentInst);
                    currentCyclePaths.insert({edge, {cycle}});
                    return paths;
                }
            }
        }

        for (auto edge : this->g.getIncomingEdges(currentInst))
        {
            if (edge->getDistance() > 0)
            {
                currentCyclePaths.insert({ edge, {} });
            }
        }

        for (auto edge : this->g.getOutgoingEdges(currentInst))
        {
            if (edge->getDistance() > 0)
            {
                continue;
            }

            if (isSinkForCurrentLoop(edge->getVertexDst()))
            {
                auto cur = vMap.find(currentInst);
                if (cur == vMap.end() || !cur->second)
                {
                    PathVec tmpPath;
                    tmpPath.length = currentLength + getLatency(currentInst);

                    auto suc = vMap.find(&edge->getVertexDst());
                    if (suc == vMap.end() || !suc->second)
                    {
                        tmpPath.push_back(&edge->getVertexDst());
                        tmpPath.length += getLatency(&edge->getVertexDst()) + edge->getDelay();

                        for (auto& keyVal : currentCyclePaths)
                        {
                            if (keyVal.first->getVertexSrc().getId() == edge->getVertexDst().getId())
                            {
                                PathVec tmpVec;
                                tmpVec.push_back(&edge->getVertexDst());
                                tmpVec.push_back(currentInst);
                                tmpVec.length = getLatency(currentInst) + edge->getDelay() + getLatency(&edge->getVertexDst());
                                keyVal.second.push_back(tmpVec);
                            }
                        }
                    }

                    for (auto& keyVal : currentCyclePaths)
                    {
                        if (keyVal.first->getVertexSrc().getId() == currentInst->getId())
                        {
                            PathVec tmpVec;
                            tmpVec.push_back(currentInst);
                            tmpVec.length = getLatency(currentInst);
                            keyVal.second.push_back(tmpVec);
                        }

                        for (auto& vec : keyVal.second)
                        {
                            if (!vec.empty() && vec.back()->getId() != keyVal.first->getVertexDst().getId())
                            {
                                vec.push_back(currentInst);
                                vec.length += getLatency(currentInst) + edge->getDelay();
                            }
                        }
                    }

                    tmpPath.push_back(currentInst);
                    paths.push_back(tmpPath);
                }
            }
            else
            {
                bool inserted = vMap.emplace( currentInst, true ).second;
                std::vector<PathVec> tmpPaths;
                if (inserted || !vMap.at(currentInst))
                {
                    if (!inserted)
                    {
                        vMap.at(currentInst) = true;
                    }
                    tmpPaths = getPathsAndCycles(&edge->getVertexDst(), currentLength + getLatency(currentInst) + edge->getDelay(), currentCyclePaths, vMap);
                    vMap.at(currentInst) = false;

                    for (auto& path : tmpPaths)
                    {
                        path.push_back(currentInst);
                    }
                    paths.insert(paths.end(), tmpPaths.begin(), tmpPaths.end());

                    for (auto& keyVal : currentCyclePaths)
                    {
                        if (keyVal.first->getVertexSrc().getId() == currentInst->getId())
                        {
                            PathVec tmpVec;
                            tmpVec.push_back(currentInst);
                            tmpVec.length = getLatency(currentInst);
                            keyVal.second.push_back(tmpVec);
                        }
                        else
                        {
                            for (auto& vec : keyVal.second)
                            {
                                if (!vec.empty() && vec.back()->getId() != keyVal.first->getVertexDst().getId())
                                {
                                    vec.push_back(currentInst);
                                    vec.length += getLatency(currentInst) + edge->getDelay();
                                }
                            }
                        }
                    }
                }
            }
        }
        return paths;
    }

    void HatScheT::NonIterativeModuloScheduler::scheduleInit()
    {
	    secondObjectiveOptimal = false;
        if (custom)
        {
            std::vector<PathVec> forwardPaths;
            std::map<Vertex*, bool> vMap;
            std::map<Edge*, std::vector<PathVec>> currentCyclePaths;
            //int tmpCount = 0;
            for (auto inst : this->g.Vertices())
            {
                if (isSourceForCurrentLoop(*inst))
                {
                    //std::cout << tmpCount++ << std::endl;
                    std::map<Edge*, std::vector<PathVec>> tmpCyclePaths;
                    {
                        std::vector<PathVec> tmpPaths = getPathsAndCycles(inst, 0, tmpCyclePaths, vMap);
                        forwardPaths.insert(forwardPaths.end(), tmpPaths.begin(), tmpPaths.end());
                    }
                    currentCyclePaths.insert(tmpCyclePaths.begin(), tmpCyclePaths.end());
                }
            }
            std::cout << "ccp: " << currentCyclePaths.size() << std::endl;

            for (auto& keyVal : currentCyclePaths)
            {
                //std::cout << "cyclePath for " << keyVal.first->getVertexSrcName() << " to " << keyVal.first->getVertexDstName() << ": [";
                for (auto& cycle : keyVal.second)
                {
                    for (auto inst : cycle)
                    {
                        //std::cout << inst->getName() << ", ";
                        this->wMap.insert({ inst, OrderedWeight(-1, true) });
                    }
                    //std::cout << "]" << std::endl;
                    this->cyclePaths.emplace_back(keyVal.first, cycle);
                }
                //this->cyclePaths.insert(cyclePaths.end(), keyVal.second.begin(), keyVal.second.end());
            }

            std::sort(forwardPaths.begin(), forwardPaths.end());
            std::vector<Vertex*> tmpOrderedForwardInstructions;
            std::cout << "fp: " << forwardPaths.size() << std::endl;
            //std::cout << "dMap: " << dMap.size() << std::endl;

            for (const auto& path : forwardPaths)
            {
                for (auto inst : path)
                {
                    if (this->wMap.insert({ inst, OrderedWeight(-path.length, false) }).second)
                    {
                        tmpOrderedForwardInstructions.push_back(inst);
                    }
                }
            }

            for (auto inst : tmpOrderedForwardInstructions)
            {
                std::vector<Vertex*> toAdd = establishTopologicalOrderForPath(inst, this->wMap);
                this->orderedForwardInstructions.insert(this->orderedForwardInstructions.end(), toAdd.begin(), toAdd.end());
            }

            //std::cout << "ofi: [";
            //for (auto inst : this->orderedForwardInstructions)
            //{
            //    std::cout << inst->getName() << ", ";
            //}
            //std::cout << "]" << std::endl;

            for (auto& keyVal : currentCyclePaths)
            {
                for (auto& vec : keyVal.second)
                {
                    for (auto inst : vec)
                    {
                        OrderedWeight& val = this->wMap.at(inst);
                        if (val.ordered)
                        {
                            val.ordered = false;
                        }
                    }
                }
            }

            std::cout << "ofi: " << this->orderedForwardInstructions.size() << std::endl;
            std::cout << "cp: " << this->cyclePaths.size() << std::endl;
        }
    }

HatScheT::NonIterativeModuloScheduler::NonIterativeModuloScheduler(HatScheT::Graph &graph,
                                                                   HatScheT::ResourceModel &resourceModel, double II)
        : IterativeModuloSchedulerLayer(graph, resourceModel, II) {}
