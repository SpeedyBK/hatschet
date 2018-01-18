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
    Edge(Vertex &src, Vertex &dst);

    Vertex& getVertexSrc(){ return Vsrc; }
    Vertex& getVertexDst(){ return Vdst; }

    const string getVertexSrcName() const {return this->Vsrc.getName();}
    const string getVertexDstName() const {return this->Vdst.getName();}

    const double getDelay() const { return delay; }
    void setDelay(double delay){ this->delay = delay; }

    const int getID() const {return this->id;}
    void setID(int id){this->id = id;}

    const bool getBackward() const {return this->backward;}
    void setBackward(bool b) {this->backward = b;}
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
    bool backward;

    Vertex &Vsrc;
    Vertex &Vdst;

    double delay;
};
ostream& operator<<(ostream& os, const Edge& e);
}
