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

#include <HatScheT/utility/subgraphs/OccurrenceSetCombination.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
OccurrenceSetCombination::OccurrenceSetCombination(Graph *g)
{
  this->g = g;
}

bool OccurrenceSetCombination::addOccurrenceSet(OccurrenceSet *occs)
{
  if(occs->getGraph() != this->g){
    cout << "OccurrenceSetCombination.addOccurrenceSet: WARNING tried to add an occurrenceSet from a different graph!" << endl;
    return false;
  }

  if(this->occsComb.size()>0){
    //check whether occS is new
    const bool is_in = this->occsComb.find(occs) != this->occsComb.end();
    if(is_in==true){
      cout << "OccurrenceSetCombination.addOccurrenceSet: WARNING tried to add an already existing occurrenceSet!" << endl;
      return false;
    }

    //check whether there exist any conflicts with already added occurrences
    for(auto it:this->occsComb){
      OccurrenceSet* occsIter = it;
      if(Utility::occurenceSetsAreConflictFree(occs,occsIter)==false){
        cout << "OccurrenceSetCombination.addOccurrenceSet: WARNING tried to add an occurrenceSet that has a conflict with an aleady added one!" << endl;
        return false;
      }
    }
  }

  this->occsComb.insert(occs);

  return true;
}

}
