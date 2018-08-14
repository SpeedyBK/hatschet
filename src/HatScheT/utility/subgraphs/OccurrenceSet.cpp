/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include <HatScheT/utility/subgraphs/OccurrenceSet.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
OccurrenceSet::OccurrenceSet(Graph *g)
{
  this->g = g;
}

bool OccurrenceSet::addOccurrence(Occurrence *occ)
{
  if(occ->getGraph() != this->g){
    cout << "OccurrenceSet.addOccurrence: WARNING tried to add an occurrence from a different graph!" << endl;
    return false;
  }

  if(this->occurrences.size()>0){
    //check whether occ is new
    const bool is_in = this->occurrences.find(occ) != this->occurrences.end();
    if(is_in==true){
      cout << "OccurrenceSet.addOccurrence: WARNING tried to add an already existing occurrence!" << endl;
      return false;
    }

    //check whether there exist conflicts with existing occurrences
    for(auto it:this->occurrences){
      Occurrence* iterOcc = it;
      if(Utility::occurrencesAreConflictFree(occ,iterOcc)==false){
        cout << "OccurrenceSet.addOccurrence: WARNING tried to add an occurrence that has a conflict with an aleady added one!" << endl;
        return false;
      }
    }
  }

  this->occurrences.insert(occ);

  return true;
}

}
