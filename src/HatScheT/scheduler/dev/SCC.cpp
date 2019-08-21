//
// Created by bkessler on 21/08/19.
//

#include "SCC.h"

HatScheT::SCC::SCC() {
  this->_type = unknown;
  this->maxVertexId = 0;
  this->name = "";
  this->id = 0;
}

//Getter Functions:

HatScheT::scctype HatScheT::SCC::getSccType() { return _type; }

int HatScheT::SCC::getId() { return this->id; }


//Setter Functions:

void HatScheT::SCC::setId(int newid) { this->id = newid; }


