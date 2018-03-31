#include <iostream>
#include <string.h>

#include <HatScheT/Exception.h>
#include <HatScheT/Graph.h>
#include <HatScheT/utility/reader/GraphMLGraphReader.h>
#include <HatScheT/utility/reader/GraphMLResourceReader.h>
#include <HatScheT/utility/writer/DotWriter.h>
#include <HatScheT/MoovacMinRegScheduler.h>
#include "HatScheT/utility/Tests.h"
/**
 * Returns the value as string of a command line argument in syntax --key=value
 * @param argv the command line string
 * @param parameter the name of the parameter
 * @param value as string if parameter string was found
 * @return True if parameter string was found
 */
bool getCmdParameter(char* argv, const char* parameter, char* &value)
{
  if(strstr(argv, parameter))
  {
    value = argv+strlen(parameter);
    return true;
  }
  else
  {
    return false;
  }
}

void print_short_help()
{
    std::cout << "usage: hatschet [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    std::cout << "General Options:" << std::endl;
    std::cout << "Option                                  Meaning" << std::endl;
    std::cout << "--solver=[Gurobi|CPLEX|SCIP|LPSolve]    ILP solver (if available in ScaLP library), also a comma separated wish list of solvers can be provided" << std::endl;
    std::cout << "--timeout=[int]                         ILP solver Timeout in seconds" << std::endl;
    std::cout << "--threads=[int]                         Number of threads for the ILP solver" << std::endl;
    std::cout << "--resource=[string]                     Path to graphML Resource File you want to read. (Make sure XercesC is enabled)" << std::endl;
    std::cout << "--graph=[string]                        Path to graphML Graph File you want to read. (Make sure XercesC is enabled)" << std::endl;
    std::cout << "--dot=[string]                          Path to you desired dot file of your graph+resource model" << std::endl;
    std::cout << std::endl;

}
int main(int argc, char *args[])
{
  std::string ilpSolver="";
  int threads=0;
  int timeout=-1; //default -1 means no timeout
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

  //parse command line
  for (int i = 1; i < argc; i++) {
    char* value;
    if(getCmdParameter(args[i],"--timeout=",value))
    {
      timeout = atol(value);
    }
    else if(getCmdParameter(args[i],"--threads=",value))
    {        
      threads = atol(value);
    }   
    else if(getCmdParameter(args[i],"--solver=",value))
    {
      ilpSolver = std::string(value);
    }
    else if(getCmdParameter(args[i],"--resource=",value))
    {
      #ifdef USE_XERCESC
      string str = std::string(value);
      try
      {       
        rm = readerRes.readResourceModel(str.c_str());
      }
      catch(HatScheT::Exception &e)
      {
        std::cerr << "Error: " << e << std::endl;
      }

      #else
      cout << "Warning: You chose the graphml parameter, but XerseC was not found. Make to to prove the path to XercesC as CMAKE_PREFIX_PATH" << endl;
      #endif
    }
    else if(getCmdParameter(args[i],"--graph=",value))
    {
      #ifdef USE_XERCESC
      string str = std::string(value);
      try
      {
        if(rm.isEmpty() == false)
        {
          HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
          g = readerGraph.readGraph(str.c_str());
          cout << g << endl;
          cout << rm << endl;
        }

        else
        {
          cout << "Empty Resource Model Provided for graph parsing! Provide a valid resource model using  --graphmlrespath=" << endl;
        }


      }
      catch(HatScheT::Exception &e)
      {
        std::cerr << "Error: " << e << std::endl;
      }

      #else
      cout << "Warning: You chose the graphml parameter, but XerseC was not found. Make to to prove the path to XercesC as CMAKE_PREFIX_PATH" << endl;
      #endif
    }
    else if(getCmdParameter(args[i],"--dot=",value))
    {
      if(rm.isEmpty() == false && g.isEmpty() == false)
      {
        cout << "Writing to dotfile " << value << ".dot" << endl;
        HatScheT::DotWriter dw(value, &g, &rm);
        dw.setDisplayNames(true);
        dw.write();
      }    
    }
    //HatScheT Auto Test Function
    else if(getCmdParameter(args[i],"--test=",value))
    {
      string str = std::string(value);
      if(str=="READ" && HatScheT::Tests::readTest()==false) exit(-1);
      if(str=="MOOVAC" && HatScheT::Tests::moovacTest()==false) exit(-1);
      if(str=="API" && HatScheT::Tests::apiTest()==false) exit(-1);
    }
    else
    {
      std::cout << "Error: Illegal Option: " << args[i] << std::endl;
      print_short_help();
      exit(-1);
    }

  }

  std::cout << "settings:" << std::endl;
  std::cout << "timeout=" << timeout << std::endl;
  std::cout << "threads=" << threads << std::endl;

  return 0;
}
