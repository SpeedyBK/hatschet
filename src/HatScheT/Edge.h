#pragma once

#include "Vertex.h"

namespace HatScheT
{

/*!
 * \brief Implementation of a simple edge used in a graph (@see Graph)
 */
class Edge
{
public:
    Edge(int id, Vertex &src, Vertex &dst);

    Vertex& getVertexSrc(){ return Vsrc; }
    Vertex& getVertexDst(){ return Vdst; }

    double getDelay(){ return delay; }
    void setDelay(double delay){ this->delay = delay; }

    //ToDo: add distance (no idea what it means in the UML)

    /*!
     * \brief < operator used for map container
     */
    bool operator<(const Edge& eref) const
    {
      return id < eref.id;
    }

protected:
    int id;

    Vertex &Vsrc;
    Vertex &Vdst;

    double delay;
};

}
