#pragma once

#include "../Graph.h"
#include "../ResourceModel.h"

namespace HatScheT
{
/*!
 * \brief The Reader class use this class as base for reading into HatScheT
 */
class Reader
{
public:
  /*!
     * \brief graphReader
     */
  Reader();
  ~Reader();
  /*!
     * \brief readGraph this function will generate a new graph object instance
     * afterwards the respective parseGraph function of a derived class is called
     * for parsing/filling the graph object
     * \return a reference to the read Graph is returned
     */
  virtual Graph& readGraph(const char* path)=0;

  virtual ResourceModel& readResourceModel(const char* path, Graph& g)=0;
protected:
  /*!
     * \brief g used to add elemets to the graph during parsing process
     */
  Graph g;
private:

};

}

