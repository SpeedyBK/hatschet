#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{

MoovacScheduler::MoovacScheduler(Graph &g, std::list<std::string>  solverWishlist, ResourceModel &resourceModel, unsigned int minII, unsigned int maxII) : SchedulerBase(g), ILPSchedulerBase(solverWishlist), ResourceConstrainedSchedulerBase(resourceModel)
{
  this->minII = minII;
  this->maxII = maxII;
  this->SLMax = 0;
}

void MoovacScheduler::resetContainer()
{
  this->t_vector.resize(0);
  this->m_container.clear();
  this->regVector.resize(0);
  this->r_container.clear();
  this->t_vectorIndices.clear();
  this->reg_vectorIndices.clear();
}

void MoovacScheduler::setUpSolverSettings()
{
  // disable solver output
  if(this->solverQuiet) solver->quiet = true;
  else solver->quiet = false;

  //set solver timeout
  solver->timeout = this->solverTimeout;
  solver->threads = this->threads;
}

void MoovacScheduler::constructProblem()
{
  //this->SLMax = this->g.getNumberOfVertices() * ( this->g.getMaxLatency() + 1);

  this->setTVectorVariables();
  this->fillRegVector();

  //solver->addConstraint(...);
}

void MoovacScheduler::schedule()
{
  this->timeoutCounter = 0;
  this->totalTime = 0;
  this->II = this->minII;

  while((this->II <= this->maxII) && (this->schedFound == false))
  {
    //cleanup
    this->resetContainer();

    //setup solver
    this->setUpSolverSettings();

    //construct the problem
    this->constructProblem();

    //solve it
    solver->solve();

    if(this->schedFound == false) this->II++;
  }



  //fill solution structure
  //startTimes[...] = ...
}

void MoovacScheduler::setTVectorVariables()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it)
  {
    Vertex* v = *it;
    int id = v->getId();

    //17
     //t_vector.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id),0,this->SLMax - v->getLatency()));
    //store tIndex
    this->t_vectorIndices.insert(make_pair(v,t_vector.size()-1));

    //16
    //this->solver->addConstraint(t_vector.back() + v->getLatency() <= this->SLMax);
  }
}

void MoovacScheduler::fillRegVector()
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;

    Vertex& srcV = e->getVertexSrc();
    Vertex& dstV = e->getVertexDst();

    //skip constant tail vertices for registers after them
    //if(this->isSourceVertex(tail) == true) continue;

    //infinity is limited might cause overflow during solving process
    this->regVector.push_back(ScaLP::newIntegerVariable("n_" + std::to_string(srcV.getId()) + "_" + std::to_string(dstV.getId()) +"_a" + to_string(e->getID()),0,1000));
    this->reg_vectorIndices.insert(make_pair(e, this->regVector.size() - 1));
  }
}

void MoovacScheduler::setGeneralConstraints()
{
    /*
    this->setSourceVerticesToZero(s);
    this->setInputIIConstraints(s);

    //5
    for(uint i = 0; i < this->graphToSchedule->getNoOfArcs(); i++)
    {
        Arc* a = this->graphToSchedule->getArcByVectorIndex(i);
        Vertex* tail = a->getTail();
        uint tailTVecIndex = tail->getTIndex();
        Vertex* head = a->getHead();
        uint headTVecIndex = head->getTIndex();

        //standard moovac formulation
        if(this->considerAlgorithmicDelays == false)
        {
            bool b = a->getBackward();
            uint bII = b*this->II;
            uint tailLatency = tail->getLatency();
            uint Di = tailLatency+a->getLatency();

            s.addConstraint(t_vector[tailTVecIndex] - t_vector[headTVecIndex] <= (int)(bII - Di));
        }

        //enhancement for DSP applications without retiming
        if(this->considerAlgorithmicDelays == true)
        {
            //standard formulation using simulink
            if(this->graphToSchedule->getHasOrigamiSystem() == true)
            {
                s.addConstraint(this->II*(a->getLatency()) - t_vector[tailTVecIndex] + t_vector[headTVecIndex] - tail->getLatency() >= 0);
            }
            //workaround for graphml Benchmark set
            else if(this->graphToSchedule->getHasOrigamiSystem() == false)
            {
                bool b = a->getBackward();

                if(b == true)
                {
                    s.addConstraint(this->II - t_vector[tailTVecIndex] + t_vector[headTVecIndex] - tail->getLatency() - a->getLatency() >= 0);
                }
                else if(b == false)
                {
                    s.addConstraint(t_vector[headTVecIndex] - t_vector[tailTVecIndex] - tail->getLatency() - a->getLatency() >= 0);
                }
            }
        }

        //enhancement for register minimization
        if(a->getRegVectorIndex() != -1 && a->getDataFlow() == true)
        {
            //standard formulation using simulink
            if(this->graphToSchedule->getHasOrigamiSystem() == true)
            {
                s.addConstraint(this->II*a->getLatency() - t_vector[tailTVecIndex] + t_vector[headTVecIndex]
                            - tail->getLatency() - this->regVector[a->getRegVectorIndex()] == 0);
            }
            //workaround for graphml Benchmark set
            else if(this->graphToSchedule->getHasOrigamiSystem() == false)
            {
                bool b = a->getBackward();

                if(b == true)
                {
                    s.addConstraint(this->II*(a->getLatency()) - t_vector[tailTVecIndex] + t_vector[headTVecIndex]
                                - tail->getLatency() - this->regVector[a->getRegVectorIndex()] == 0);
                }
                else if(b == false)
                {

                    s.addConstraint(t_vector[headTVecIndex] - t_vector[tailTVecIndex]
                                - tail->getLatency() - a->getLatency() - this->regVector[a->getRegVectorIndex()] == 0);

                }
            }
        }
    }*/
}

void MoovacScheduler::setModuloAndResourceConstraints()
{
    /*unsigned int resourceCount = this->graphToSchedule->getNoOfResources(); //R

    m_container.resize(0);

    for(unsigned int i = 0; i < resourceCount; i++)
    {
        //declare r-vector
        vector<ScaLP::Variable> r_vector;
        //declare y-vector
        vector<ScaLP::Variable> y_vector;
        //declare m-vector
        vector<ScaLP::Variable> m_vector;
        //declare eps-matrix
        vector<vector<ScaLP::Variable> > eps_matrix;
        //declare mu-matrix
        vector<vector<ScaLP::Variable> > mu_matrix;

        Resource* r = this->graphToSchedule->getResourceByVecIndex(i);
        if(this->graphToSchedule->getNoOfOpsUsingResource(r->getId())==0) continue;

        int resourceId = r->getId();
        int ak = r->getLimit();
        multimap<int, Vertex*> L = this->graphToSchedule->getL();

        std::pair <std::multimap<int,Vertex*>::iterator, std::multimap<int,Vertex*>::iterator> Lk;
        Lk = L.equal_range(resourceId);

        for (std::multimap<int,Vertex*>::iterator it=Lk.first; it!=Lk.second; ++it)
        {
            Vertex* v1 = (*it).second;
            int tIndex = v1->getTIndex();
            //18
            r_vector.push_back(ScaLP::newIntegerVariable("r_" + std::to_string(v1->getId()),0,10000));
            //19
            m_vector.push_back(ScaLP::newIntegerVariable("m_" + std::to_string(v1->getId()),0,10000));
            //20
            y_vector.push_back(ScaLP::newIntegerVariable("y_" + std::to_string(v1->getId()),0,10000));

            //13
            s.addConstraint(this->t_vector[tIndex] - y_vector.back()*this->II - m_vector.back() == 0);
            //14
            s.addConstraint(r_vector.back() <= r->getLimit() - 1);
            //15
            s.addConstraint(m_vector.back() <= this->II - 1);

            //store location of the r variables in the vertices
            v1->setRVectorIndex(m_vector.size() - 1);
            v1->setRContainerIndex(this->m_container.size());

            //declare eps-vector
            vector<ScaLP::Variable> eps_vector;
            //declare mu-vector
            vector<ScaLP::Variable> mu_vector;

            for (std::multimap<int,Vertex*>::iterator it2=Lk.first; it2!=Lk.second; ++it2)
            {
                Vertex* v2 = (*it2).second;

                if(v1 != v2)
                {
                    eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
                    mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
                }

                else if(v1==v2)
                {
                    eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
                    mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
                }
            }

            eps_matrix.push_back(eps_vector);
            mu_matrix.push_back(mu_vector);
        }

        this->r_container.push_back(r_vector);
        this->m_container.push_back(m_vector);
        y_container.push_back(y_vector);
        eps_container.push_back(eps_matrix);
        mu_container.push_back(mu_matrix);

        if(eps_matrix.size() > 1)
        {
            for(uint j = 0; j < eps_matrix.size(); j++)
            {
                for(uint k = 0; k < eps_matrix.size(); k++)
                {
                    if(k!=j && j<k)
                    {
                        //6
                        s.addConstraint(eps_matrix[j][k] + eps_matrix[k][j] <= 1);
                        //12
                        s.addConstraint(eps_matrix[j][k] + eps_matrix[k][j] + mu_matrix[j][k] + mu_matrix[k][j] >= 1);

                    }

                    if(k!=j)
                    {
                        //7
                        s.addConstraint(r_vector[j]-r_vector[k] - (ak*eps_matrix[j][k]) + ak >= 1);
                        //8
                        s.addConstraint(r_vector[j]-r_vector[k] - (ak*eps_matrix[j][k]) <= 0);
                        //10
                        s.addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) + this->II >= 1);
                        //11
                        s.addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) <= 0);
                    }
                }
            }
        }
    }*/
}

}
