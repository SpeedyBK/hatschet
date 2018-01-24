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
		enum DependencyType { Data, Precedence };

    Edge(Vertex &src, Vertex &dst, int delay=0, bool backward=false, DependencyType dependencyType=Data);

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
		DependencyType dependencyType;
    int delay;

    Vertex &Vsrc;
    Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
