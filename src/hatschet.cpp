#include <iostream>
#include <string.h>

#include <HatScheT/Exception.h>
#include <HatScheT/ASAPScheduler.h>
#include <HatScheT/Graph.h>
#include <HatScheT/graphReader/graphMLReader.h>
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
    std::cout << "--graphmlpath=[string]                  Path to graphML File you want to read. (Make sure XercesC is enabled)" << std::endl;
    std::cout << std::endl;

}
int main(int argc, char *args[])
{
  std::string ilpSolver="";
  int threads=0;
  int timeout=-1; //default -1 means no timeout
  
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
    else if(getCmdParameter(args[i],"--graphmlpath=",value))
    {
      #ifdef USE_XERCESC
      string str = std::string(value);
      HatScheT::GraphMLReader gmlr;
      HatScheT::Graph g = gmlr.readGraph(str.c_str());
      cout << g << endl;
      #else
      cout << "Warning: You chose the graphml parameter, but XerseC was not found. Make to to prove the path to XercesC as CMAKE_PREFIX_PATH" << endl;
      #endif
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

  try
  {
  	HatScheT::Graph g;
  	HatScheT::ASAPScheduler sched(g);	
  	
  	sched.schedule();
  	
  }
  catch(HatScheT::Exception &e)
  {
    std::cerr << "Error: " << e << std::endl;
  }

  return 0;
}
