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
      cout << "Occurrence.addEdge: WARNING tried to add an already existing occurrence!" << endl;
      return false;
    }

    //check whether there exist conflicts with existing occurrences
    for(auto it:this->occurrences){
      Occurrence* iterOcc = it;
      if(Utility::occurrencesAreConflictFree(occ,iterOcc)==false){
        cout << "Occurrence.addEdge: WARNING tried to add an occurrence that has a conflict with an aleady added one!" << endl;
        return false;
      }
    }
  }

  this->occurrences.insert(occ);

  return true;
}

}
