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
