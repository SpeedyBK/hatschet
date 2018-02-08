#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{

MoovacScheduler::MoovacScheduler(Graph &g, std::list<std::string>  solverWishlist, ResourceModel &resourceModel, unsigned int minII, unsigned int maxII) : SchedulerBase(g), ILPSchedulerBase(solverWishlist), ResourceConstrainedSchedulerBase(resourceModel)
{
  this->minII = minII;
  this->maxII = maxII;
  this->SLMax = 0;
  this->schedFound = false;
}

void MoovacScheduler::resetContainer()
{
  if(this->solver->getBackendName() != "Dynamic: Gurobi") this->solver->reset();
  this->m_container.clear();
  this->regVector.clear();
  this->r_container.clear();
  this->t_vectorIndices.clear();
  this->reg_vectorIndices.clear();
  this->t_vector.clear();
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
  //case unlimited
  if(this->maxLatencyConStraint == -1){
    this->SLMax = this->g.getNumberOfVertices() * ( this->resourceModel.getMaxLatency() + 1);
  }
  else if(this->maxLatencyConStraint > 0)
  {
    this->SLMax = this->maxLatencyConStraint;
  }
  else
  {
     throw new Exception("MoovacScheduler.constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConStraint));
  }

  this->setTVectorVariables();
  this->fillRegVector();

  //add constraints
  this->setGeneralConstraints();
  this->setModuloAndResourceConstraints();

  //set Objective
  this->setObjective();
}

void MoovacScheduler::setObjective()
{
    ScaLP::Term sum;

    for(ScaLP::Variable &v:t_vector)
    {
      sum = sum + v;
    }

    this->solver->setObjective(ScaLP::minimize(sum));
}

void MoovacScheduler::schedule()
{
  this->timeoutCounter = 0;
  this->totalTime = 0;
  this->II = this->minII;

  while(this->II <= this->maxII)
  {
    //cleanup
    this->resetContainer();

    if(this->solver->getBackendName() == "Dynamic: Gurobi") this->solver = new ScaLP::Solver({"Gurobi"});

    //setup solver
    this->setUpSolverSettings();

    //construct the problem
    this->constructProblem();

    //solve it
    ScaLP::status stat = this->solver->solve();

    if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE)
    {
      this->schedFound = true;
    }
    else
    {
        this->schedFound = false;
    }

    if(this->schedFound == false) (this->II)++;
    else if(this->schedFound == true) break;
  }

  if(this->schedFound == true)
  {
    std::cout<< "Schedule found! II is " << this->II << std::endl;
    std::cout<< this->solver->getResult() << std::endl;
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
     t_vector.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id),0,this->SLMax - this->resourceModel.getVertexLatency(v)));
    //store tIndex
    this->t_vectorIndices.insert(make_pair(v,t_vector.size()-1));

    //16
    this->solver->addConstraint(t_vector.back() + this->resourceModel.getVertexLatency(v) <= this->SLMax);
  }
}

void MoovacScheduler::fillRegVector()
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;

    Vertex& srcV = e->getVertexSrc();
    Vertex& dstV = e->getVertexDst();

    //skip input tail vertices for registers after them
    if(this->g.isSourceVertex(&srcV) == true) continue;

    //infinity is limited might cause overflow during solving process
    this->regVector.push_back(ScaLP::newIntegerVariable("n_" + std::to_string(srcV.getId()) + "_" + std::to_string(dstV.getId()) +"_a" + to_string(e->getID()),0,1000));
    this->reg_vectorIndices.insert(make_pair(e, this->regVector.size() - 1));
  }
}

void MoovacScheduler::setSourceVerticesToZero()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it)
  {
    Vertex* v = *it;

    unsigned int index =this->t_vectorIndices.at(v);
    ScaLP::Variable  temp = this->t_vector[index];

    if(this->g.isSourceVertex(v)) this->solver->addConstraint(temp  == 0);
  }
}

void MoovacScheduler::setGeneralConstraints()
{
  this->setSourceVerticesToZero();
  //useless?
  //this->setInputIIConstraints(s);

 //5
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->t_vectorIndices.at(src);
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->t_vectorIndices.at(dst);

    //standard moovac formulation
    //if(this->considerAlgorithmicDelays == false)
    {
      bool b = e->getBackward();
      unsigned int bII = b * this->II;
      unsigned int srcLatency = this->resourceModel.getVertexLatency(src);
      unsigned int Di = srcLatency + e->getDelay();

      this->solver->addConstraint(t_vector[srcTVecIndex] - t_vector[dstTVecIndex] <= (int)(bII - Di));
    }

    //enhancement for DSP applications without retiming
    /* if(this->considerAlgorithmicDelays == true)
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
    }*/
  }
}

void MoovacScheduler::setModuloAndResourceConstraints()
{
    //unsigned int resourceCount = this->resourceModel.getNoOfResources(); //R

    this->m_container.resize(0);

    for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
    {
      Resource* r = *it;
      if(this->resourceModel.getNoOfVerticesRegisteredToResource(r)==0) continue;

      int ak = r->getLimit();
      set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);

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

      //for (std::multimap<int,Vertex*>::iterator it=Lk.first; it!=Lk.second; ++it)
      for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++)
      {
          const Vertex* v1 = (*it2);
          int tIndex = this->t_vectorIndices.at(v1);
          //18
          r_vector.push_back(ScaLP::newIntegerVariable("r_" + std::to_string(v1->getId()),0,10000));
          //19
          m_vector.push_back(ScaLP::newIntegerVariable("m_" + std::to_string(v1->getId()),0,10000));
          //20
          y_vector.push_back(ScaLP::newIntegerVariable("y_" + std::to_string(v1->getId()),0,10000));

          //13
          this->solver->addConstraint(this->t_vector[tIndex] - y_vector.back()*this->II - m_vector.back() == 0);
          //14
          this->solver->addConstraint(r_vector.back() <= r->getLimit() - 1);
          //15
          this->solver->addConstraint(m_vector.back() <= this->II - 1);

          //store location of the r variables in the vertices
          //v1->setRVectorIndex(m_vector.size() - 1);
         // v1->setRContainerIndex(this->m_container.size());

          //declare eps-vector
          vector<ScaLP::Variable> eps_vector;
          //declare mu-vector
          vector<ScaLP::Variable> mu_vector;

          //for (std::multimap<int,Vertex*>::iterator it2=Lk.first; it2!=Lk.second; ++it2)
          for(set<const Vertex*>::iterator it3 = verOfRes.begin(); it3 != verOfRes.end(); it3++)
          {
              const Vertex* v2 = (*it3);

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
                      this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] <= 1);
                      //12
                      this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] + mu_matrix[j][k] + mu_matrix[k][j] >= 1);

                  }

                  if(k!=j)
                  {
                      //7
                      this->solver->addConstraint(r_vector[j]-r_vector[k] - (ak*eps_matrix[j][k]) + ak >= 1);
                      //8
                      this->solver->addConstraint(r_vector[j]-r_vector[k] - (ak*eps_matrix[j][k]) <= 0);
                      //10
                      this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) + this->II >= 1);
                      //11
                      this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) <= 0);
                  }
              }
          }
      }
  }
}

}
