/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Edge.h"

namespace HatScheT
{

Edge::Edge(Vertex &src, Vertex &dst, int distance, DependencyType dependencyType) : dependencyType(dependencyType), distance(distance), Vsrc(src), Vdst(dst)
{
}

ostream& operator<<(ostream& os, const Edge& e)
{
  string datatype = "Data";
  if(e.getDependencyType()==Edge::DependencyType::Precedence) datatype = "Precedence";
   os << "Edge Id: " << e.getID() << ". From Node " << e.getVertexSrcName()<< " to " << e.getVertexDstName()
      << " (Delay " << e.getDelay() << " Distance " << e.getDistance() << ")" << " depType " << datatype;

  return os;
}

bool Edge::isDataEdge()
{
  if(this->dependencyType==DependencyType::Data) return true;
  return false;
}

}
