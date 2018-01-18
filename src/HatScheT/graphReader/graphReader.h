#pragma once

#include "../Graph.h"

namespace HatScheT
{
/*!
 * \brief The graphReader class use this class as base for graph reading into HatScheT
 */
class GraphReader
{
public:
  /*!
     * \brief graphReader
     */
  GraphReader();
  ~GraphReader();
  /*!
     * \brief readGraph this function will generate a new graph object instance
     * afterwards the respective parseGraph function of a derived class is called
     * for parsing/filling the graph object
     * \return a reference to the read Graph is returned
     */
  virtual Graph& readGraph(const char* path)=0;
protected:
  /*!
     * \brief g used to add elemets to the graph during parsing process
     */
  Graph g;
private:
  /*!
     * \brief parseGraph base function of the main parsing function for graph objects
     * \param path
     * \param readGraph
     * \return
     */

};

}

