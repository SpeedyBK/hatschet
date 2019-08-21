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

    //Setter Functions:
    void setId (int id);

  private:
    int id;
    scctype _type;
  };

}
#endif //HATSCHET_SCC_H
