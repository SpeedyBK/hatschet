//
// Created by bkessler on 21/08/19.
//

#ifndef HATSCHET_SCC_H
#define HATSCHET_SCC_H

#pragma once

#include <HatScheT/Graph.h>

namespace HatScheT {

  enum scctype {unknown, basic, complex, trivial};

  class SCC : public Graph {
  public:
    SCC();

    //Getter Functions:
    scctype getSccType();
    int getId ();
    map <Vertex*, Vertex*> getVertexMap();


    //Setter Functions:
    void setId (int id);
    void setSCCType(scctype sT);

    void createVertexMap(Vertex* V);


    void printVertexMap();

  private:
    int id;
    scctype _type;

    map <Vertex*, Vertex*> vertexMap;

  };

}
#endif //HATSCHET_SCC_H
