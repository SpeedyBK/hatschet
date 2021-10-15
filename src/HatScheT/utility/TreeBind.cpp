#include "TreeBind.h"
#include <HatScheT/utility/Exception.h>
#include <stdexcept>
#include <chrono>
#include <algorithm>
#include <sstream>

namespace HatScheT {
  TreeBind::TreeBind(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments) 
  : g(g), rm(rm), sched(sched), II(II), portAssignments(portAssignments), currentBinding(), bestBinding(), timeBudget(300.0), wMux(1.0), wReg(1.0), status("NOT_SOLVED"), quiet(true) {
    // check if port assignments are complete
    for(auto e : g->Edges()) {
      try {
        // chaining edges do not need a port assignment
        if (!e->isDataEdge()) continue; 
        // check if port assignment is set
        portAssignments.at(e);
      }
      catch (std::out_of_range&) {
        // throw error for missing port assignment
        auto vSrc = e->getVertexSrcName();
        auto vDst = e->getVertexDstName();
        throw HatScheT::Exception("Missing port assignment for edge '"+vSrc+"' -> '"+vDst+"'");
      }
    }
    
    // compute queue for tree traversal
    // at the moment we just insert all limited operations
    // in order of appearance in memory
    for (auto r : rm->Resources()) {
      if (r->isUnlimited()) continue;
      auto vs = rm->getVerticesOfResource(r);
      for (auto v : vs) {
        // omit const keyword because it is annoying
        this->queue.push_back(const_cast<Vertex*>(v));
      }
    }
    
    // compute FU queues for each limited operation
    // at the moment just try 0, ... , #FUs-1 in ascending order
    for (auto r : rm->Resources()) {
      if (r->isUnlimited()) continue;
      auto numFUs = r->getLimit();
      auto vs = rm->getVerticesOfResource(r);
      for (auto vc : vs) {
        // omit const keyword because it is annoying
        auto v = const_cast<Vertex*>(vc);
        for (int limit=0; limit<numFUs; limit++) {
          this->resourceQueues[v].push_back(limit);
        }
        this->lastTriedBinding[v] = this->resourceQueues[v].back();
      }
    }
  }

  void TreeBind::setQuiet(bool q) {
  	this->quiet = q;
  }
  
  void TreeBind::setRegCostFactor(double wReg) {
    this->wReg = wReg;
  }
  
  void TreeBind::setMuxCostFactor(double wMux) {
    this->wMux = wMux;
  }
  
  void TreeBind::setTimeout(unsigned int timeout) {
    this->timeBudget = (double) timeout;
  }
  
  void TreeBind::bind() {
    // store info about edges for efficient lookup
    for (auto &e : this->g->Edges()) {
      if (!e->isDataEdge()) continue;
      auto *vSrc = &e->getVertexSrc();
      auto *vDst = &e->getVertexDst();
      this->outgoingEdges[vSrc].push_back(e);
      this->incomingEdges[vDst].push_back(e);
    }
    
    // init ScaLP status
    this->status = "TIMEOUT_INFEASIBLE";

    // fix binding for operations with unlimited resources
    for (auto r : this->rm->Resources()) {
      if (!r->isUnlimited()) continue;
      int resourceCounter = 0;
      auto vs = rm->getVerticesOfResource(r);
      for (auto v : vs) {
        this->currentBinding.resourceBindings[v->getName()] = resourceCounter++;
        if (!this->quiet) {
        	std::cout << "TreeBind::bind: fixed binding '" << v->getName() << "' -> '" << this->currentBinding.resourceBindings[v->getName()] << "'" << std::endl;
        }
      }
    }

    // fix connections between unlimited operations
    // and init binding costs
    // even though they will never produce multiplexers
    // but they might produce registers
		this->currentBinding.multiplexerCosts = 0;
		this->currentBinding.registerCosts = 0;
    for (auto &rSrc : this->rm->Resources()) {
      if (!rSrc->isUnlimited()) continue;
      auto vs = rm->getVerticesOfResource(rSrc);
      for (auto &vSrc : vs) {
      	int maxLifetime = 0;
        for (auto &e : this->outgoingEdges[vSrc]) {
          auto *vDst = &e->getVertexDst();
          auto *rDst = this->rm->getResource(vDst);
          if (!rDst->isUnlimited()) continue;
					if (!this->quiet) {
						std::cout << "TreeBind::bind: found edge between unlimited resources: '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
					}
          auto fuSrc = this->currentBinding.resourceBindings.at(vSrc->getName());
          auto fuDst = this->currentBinding.resourceBindings.at(vDst->getName());
          auto lifetime = this->getLifetime(e);
          if (lifetime > maxLifetime) maxLifetime = lifetime;
          auto connectionMadness = std::make_pair(std::make_pair(std::make_pair(rSrc->getName(), fuSrc), std::make_pair(rDst->getName(), fuDst)), std::make_pair(lifetime, this->portAssignments[const_cast<Edge*>(e)]));
          if (std::find(this->currentBinding.fuConnections.begin(), this->currentBinding.fuConnections.end(), connectionMadness) == this->currentBinding.fuConnections.end()) {
						this->currentBinding.fuConnections.push_back(connectionMadness);
						this->currentBinding.multiplexerCosts++;
					}
        }
        this->currentBinding.registerCosts += maxLifetime;
				if (!this->quiet) {
					std::cout << "  updated register costs to " << this->currentBinding.registerCosts << std::endl;
				}
      }
    }
		if (!this->quiet) {
			std::cout << "TreeBind::bind: initialized binding costs for unlimited resources" << std::endl;
		}
    
    // do a full tree search for operations with limited resources ...
    this->iterativeTreeSearch();
  }
  
  Binding::BindingContainer TreeBind::getBinding() {
    return this->bestBinding;
  }
  
  void TreeBind::iterativeTreeSearch() {
  	if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: start" << std::endl;
  	}
  	// init iteration counter
		this->iterationCounter = 0;
    // init time tracker
    auto startTime = std::chrono::steady_clock::now();
    auto elapsedTime = 0.0;
    // init occupiedResources
    for (auto r : this->rm->Resources()) {
      for (int i=0; i<this->II; i++) {
        this->occupiedResources[std::make_pair(r, i)] = std::list<int>();
      }
    }
		if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: initialized time tracker" << std::endl;
		}

    // init stack
		// and push first vertex on stack based on the number of its possible resource bindings
    std::list<std::list<Vertex*>::iterator> stack;
    for (int i=0; i<this->rm->getResource(this->queue.front())->getLimit(); i++) {
      stack.push_front(this->queue.begin());
    }
		if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: initialized stack" << std::endl;
		}
    
    while (!stack.empty() and this->timeBudget > elapsedTime) {
    	// debug info
			this->iterationCounter++;
			if (!this->quiet) {
				std::cout << std::endl << "TreeBind::iterativeTreeSearch: start new iteration (" << this->iterationCounter << ")" << std::endl;
				std::cout << "  elapsed time = " << elapsedTime << std::endl;
			}

      // pop front vertex from stack
      auto currentVertexIterator = stack.front();
      auto currentVertex = *currentVertexIterator;
      //auto currentResource = this->rm->getResource(currentVertex); // O(log n)
      //auto currentScheduleTime = this->sched[currentVertex]; // O(log n)
      stack.pop_front();
			if (!this->quiet) {
				std::cout << "  next vertex: " << currentVertex->getName() << std::endl;
				std::cout << "  current resource bindings:" << std::endl;
				for (auto &it : this->currentBinding.resourceBindings) {
					std::cout << "    " << it.first << " -> " << it.second << std::endl;
				}
				if (this->currentBinding.resourceBindings.empty()) {
					std::cout << "    empty" << std::endl;
				}
				std::cout << "  current fu connections:" << std::endl;
				for (auto &it : this->currentBinding.fuConnections) {
					std::cout << "    " << it.first.first.first << " (" << it.first.first.second << ") -> " << it.first.second.first << " (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime register" << std::endl;
				}
				if (this->currentBinding.fuConnections.empty()) {
					std::cout << "    empty" << std::endl;
				}
				std::cout << "  multiplexer costs: " << this->currentBinding.multiplexerCosts << std::endl;
				std::cout << "  register costs: " << this->currentBinding.registerCosts << std::endl;
			}
      
      // remove binding info from binding for all vertices until the current one
      // obviously only if binding info already exists...
      // simultaneously update current costs
      // we must through the queue starting from the current vertex until the end to remove all
      // nodes from the tree until the end
      bool startRemoving = false;
      for (auto v : this->queue) {
      	if (v == currentVertex) {
      		startRemoving = true;
      	}
      	if (startRemoving) {
					this->removeBindingInfo(v);
					if (!this->quiet) {
						std::cout << "  removed binding info" << std::endl;
					}
      	}
      }
      
      // calculate new binding info for current node 
      // and insert it into binding container
      // simultaneously update current costs
      this->addBindingInfo(currentVertex);
			if (!this->quiet) {
				std::cout << "  added binding info" << std::endl;
			}
      
      // check if we can prune the search tree starting from here
      if (this->currentBinding.multiplexerCosts > 0 and this->currentBinding.registerCosts > 0 and this->bestBinding.multiplexerCosts > 0 and this->bestBinding.registerCosts > 0) {
        if ((this->wMux * this->currentBinding.multiplexerCosts + this->wReg * this->currentBinding.registerCosts) > (this->wMux * this->bestBinding.multiplexerCosts + this->wReg * this->bestBinding.registerCosts)) {
          std::cout << "  pruned search space" << std::endl;
        	continue;
        }
        else if (!this->quiet) {
					std::cout << "  could not prune search space" << std::endl;
				}
      }
      
      // check if we hit a leaf node
      // if so: check if we need to update best binding
      // push children on stack (push front for depth first search)
      if (currentVertex == this->queue.back()) {
				if (!this->quiet) {
					std::cout << "  hit leaf node" << std::endl;
				}
        if (
          // current binding is better than best binding
          ((this->wMux * this->currentBinding.multiplexerCosts + this->wReg * this->currentBinding.registerCosts) < (this->wMux * this->bestBinding.multiplexerCosts + this->wReg * this->bestBinding.registerCosts))
          or
          // best binding is not feasible
          (this->bestBinding.multiplexerCosts < 0 or this->bestBinding.registerCosts < 0)) {
          this->bestBinding = this->currentBinding;
          this->status = "TIMEOUT_FEASIBLE";
					std::cout << "  updated best solution" << std::endl;
        }
      }
      else {
				if (!this->quiet) {
					std::cout << "  did not hit leaf node" << std::endl;
				}
        auto nextVertexIterator = std::next(currentVertexIterator);
        auto nextVertex = *nextVertexIterator;
        auto nextResource = this->rm->getResource(nextVertex); // O(log n)
        auto nextScheduleTime = this->sched[nextVertex]; // O(log n)
        for (int i=0; i<nextResource->getLimit() - this->occupiedResources[std::make_pair(nextResource, nextScheduleTime % this->II)].size(); i++) { // O(m*n)
          stack.push_front(nextVertexIterator);
        }
      }
      
      // update elapsed time
      auto currentTime = std::chrono::steady_clock::now();
      elapsedTime = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count()) / 1000.0;
			if (!this->quiet) {
				std::cout << "  updated elapsed time (" << elapsedTime << "); starting new iteration now..." << std::endl;
			}
    }
    // we have found the optimum solution iff the stack is empty
    // i.e. we didn't terminate because of a timeout
    if (stack.empty()) {
			if (!this->quiet) {
				std::cout << "TreeBind::iterativeTreeSearch: found optimum solution" << std::endl;
			}
    	this->status = "OPTIMAL";
    }
    else if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: did not find optimum solution" << std::endl;
		}
  }
  
  void TreeBind::removeBindingInfo(Edge* e) {
		std::cout << "    removing binding info for edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
    bool removedBindingInfo = false;
		for (auto itPtr=this->currentBinding.fuConnections.begin(); itPtr!=this->currentBinding.fuConnections.end(); itPtr++) {
      auto it = *itPtr;
      // skip edges between two unlimited resources (as they are irrelevant)
      if (this->rm->getResource(it.first.first.first)->isUnlimited() and this->rm->getResource(it.first.second.first)->isUnlimited()) continue;
      // pair<pair<pair<src resource type, src FU number>, pair<dst resource type, dst FU number>>, pair<number of lifetime registers, dst input port number>>
      auto bindingEdgeMapIterator = this->bindingEdgeMap.find(it);
      if (bindingEdgeMapIterator == this->bindingEdgeMap.end()) {
      	std::stringstream con;
      	con << "'" << it.first.first.first << "' (" << it.first.first.second << ") -> '" << it.first.second.first << "' (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers";
      	throw HatScheT::Exception("could not find responsible edges for FU connection ("+con.str()+") - this should never happen...");
      }
      auto edges = (*bindingEdgeMapIterator).second;
      auto edgeIterator = std::find(edges.begin(), edges.end(), e);
      if (edgeIterator == edges.end()) continue;
      edges.erase(edgeIterator);
      if (edges.empty()) {
        // this was the only resource that was responsible 
        // for this specific connection
        // now we must erase the connection
        // i.e., handle mux costs;
        this->currentBinding.multiplexerCosts--;
        // handle register costs;
        auto &regList = this->lifetimeRegisters[std::make_pair(it.first.first.first, it.first.first.second)];
        if (regList.front().second == e) {
          this->currentBinding.registerCosts -= regList.front().first;
          regList.pop_front();
          if (!regList.empty()) {
            this->currentBinding.registerCosts += regList.front().first;
          }
					if (!this->quiet) {
						std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
					}
        }
        // handle containers
        bindingEdgeMap.erase(bindingEdgeMapIterator);
        this->currentBinding.fuConnections.erase(itPtr);
      }
      // now we can stop this loop because each edge only leads at most 
      // to one associated fu connection
      removedBindingInfo = true;
      break; 
    }
		if (!removedBindingInfo) {
			throw HatScheT::Exception("Failed to remove binding info for edge ('"+e->getVertexSrcName()+"' -> '"+e->getVertexDstName()+"' with distance '"+std::to_string(e->getDistance())+"') - that should never happen");
		}
  }
  
  void TreeBind::addBindingInfo(Edge* e) {
		std::cout << "    adding binding info for edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
    // check if there already exists a connection 
    // between the correct FUs with the correct number of lifetime 
    // registers to the correct port
    auto srcV = &e->getVertexSrc();
    auto dstV = &e->getVertexDst();
    // only continue if both vertices already have a binding
    auto srcFuIt = this->currentBinding.resourceBindings.find(srcV->getName());
    auto dstFuIt = this->currentBinding.resourceBindings.find(dstV->getName());
    if (srcFuIt == this->currentBinding.resourceBindings.end()) {
      return;
    }
    if (dstFuIt == this->currentBinding.resourceBindings.end()) {
      return;
    }
    auto srcFu = (*srcFuIt).second;
    auto dstFu = (*dstFuIt).second;
    auto srcRes = this->rm->getResource(srcV);
    auto dstRes = this->rm->getResource(dstV);
    auto lifetime = this->getLifetime(e);
    auto port = this->portAssignments[e];
    auto connectionMadness = std::make_pair(std::make_pair(std::make_pair(srcRes->getName(), srcFu), std::make_pair(dstRes->getName(), dstFu)), std::make_pair(lifetime, port));
		this->bindingEdgeMap[connectionMadness].push_back(e);
    if (std::find(this->currentBinding.fuConnections.begin(), this->currentBinding.fuConnections.end(), connectionMadness) != this->currentBinding.fuConnections.end()) {
      // connection already exists and can be re-used for this edge
      std::cout << "      connection already exists and will be reused for this edge" << std::endl;
      return;
    }
    // increment mux costs because we need one more connection
    this->currentBinding.multiplexerCosts++;
    
    // check if enough lifetime registers are available
    // and insert edge into lifetime register list
    auto &regList = this->lifetimeRegisters[std::make_pair(srcRes->getName(), srcFu)];
    if (regList.empty()) {
      // this is the first operation bound to this fu
      regList.push_front(std::make_pair(lifetime, e));
      this->currentBinding.registerCosts += lifetime;
			if (!this->quiet) {
				std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
			}
    }
    else {
      // there is at least one other operation bound to this fu
      auto availableRegs = regList.front().first;
      if (availableRegs < lifetime) {
        this->currentBinding.registerCosts += (lifetime - availableRegs);
				if (!this->quiet) {
					std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
				}
      }
      bool inserted = false;
      for (auto it = regList.begin(); it != regList.end(); it++) {
        if (it->first < lifetime) {
          regList.insert(it, std::make_pair(lifetime, e));
          inserted = true;
          break;
        }
      }
      if (!inserted) {
        regList.push_back(std::make_pair(lifetime, e));
      }
    }
    // add port connection
    this->currentBinding.fuConnections.emplace_back(connectionMadness);
  }
  
  void TreeBind::removeBindingInfo(Vertex* v) {
  	// maybe a binding for this vertex does not exist yet...
  	// skip it then
		auto mapIt = this->currentBinding.resourceBindings.find(v->getName());
		if (mapIt == this->currentBinding.resourceBindings.end()) return;

    // handle incoming edges from other FUs
    for (auto &eConst : this->incomingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
      if (!this->quiet) {
				std::cout << "    incoming edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
      }
			if (this->currentBinding.resourceBindings.find(e->getVertexSrcName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->removeBindingInfo(e);
    }
    
    // handle outgoing edges to other FUs
    for (auto &eConst : this->outgoingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    outgoing edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexDstName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->removeBindingInfo(e);
    }
    
    // remove binding variable for that vertex and update occupied resources
    auto fu = (*mapIt).second;
    this->lastTriedBinding[v] = fu;
    auto moduloSlot = this->sched[v] % this->II;
    auto res = this->rm->getResource(v);
    this->occupiedResources[std::make_pair(res, moduloSlot)].remove(fu);
    this->currentBinding.resourceBindings.erase(mapIt);
  }
  
  void TreeBind::addBindingInfo(Vertex* v) {
    // add binding variable for that vertex and update occupied resources
    auto moduloSlot = this->sched[v] % this->II;
    auto res = this->rm->getResource(v);
    auto resSlotPair = std::make_pair(res, moduloSlot);
    auto vertexQueue = this->resourceQueues[v];
    auto nextBindingIt = std::find(vertexQueue.begin(), vertexQueue.end(), this->lastTriedBinding[v]);
    for (int i=0; i<res->getLimit(); i++) {
      if (*nextBindingIt == vertexQueue.back()) {
				nextBindingIt = vertexQueue.begin();
      }
      else {
				nextBindingIt = std::next(nextBindingIt);
      }
      if (std::find(this->occupiedResources[resSlotPair].begin(), this->occupiedResources[resSlotPair].end(), *nextBindingIt) == this->occupiedResources[resSlotPair].end()) {
        this->occupiedResources[resSlotPair].push_back(*nextBindingIt);
        break;
      }
    }
    auto nextBinding = *nextBindingIt;
    this->currentBinding.resourceBindings[v->getName()] = nextBinding;
    if (!this->quiet) {
    	std::cout << "  fixed binding '" << v->getName() << "' -> '" << nextBinding << "'" << std::endl;
    }
    
    // handle incoming edges from other FUs
    for (auto &eConst : this->incomingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    incoming edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexSrcName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->addBindingInfo(e);
    }
    
    // handle outgoing edges to other FUs
    for (auto &eConst : this->outgoingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    outgoing edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexDstName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->addBindingInfo(e);
    }
  }
  
  std::string TreeBind::getSolutionStatus() {
    return this->status;
  }
  
  int TreeBind::getLifetime(const Edge* e) {
    try {
      return this->lifetimes.at(e);
    }
    catch (std::out_of_range&) {
    	auto src = &e->getVertexSrc();
    	auto dst = &e->getVertexDst();
      auto lifetime = this->sched[dst] - this->sched[src] - this->rm->getVertexLatency(src) + (this->II * e->getDistance());
      if (!this->quiet) {
      	std::cout << "    lifetime for edge '" << src->getName() << "' -> '" << dst->getName() << "' with distance '" << e->getDistance() << "' is '" << lifetime << "'" << std::endl;
      }
      this->lifetimes[e] = lifetime;
      return lifetime;
    }
  }
}
