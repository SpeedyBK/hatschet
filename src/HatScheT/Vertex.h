#pragma once
#include <iostream>
#include <string>
using namespace std;

namespace HatScheT
{

/*!
 * \brief Implementation of a simple vertex used in a graph (@see Graph)
 */
class Vertex
{
public:
  Vertex(int id);

  const int getId() const { return id; }
  /*!
     * \brief setLatency
     * \param l
     */
  void setLatency(int l){this->latency = l;}
  const int getLatency() const {return this->latency;}
  /*!
   * \brief setName
   * \param s
   */
  void setName(string s){this->name = s;}
  const string getName() const {return this->name;}
  /*!
     * \brief < operator used for map container
     */
  bool operator<(const Vertex& vref) const
  {
    return id < vref.id;
  }


protected:
  string name;
  int latency;
  const int id;
  std::string resourceType; //ToDo: Probably the type has to be changed to something else
};
ostream& operator<<(ostream& os, const Vertex& v);
}
