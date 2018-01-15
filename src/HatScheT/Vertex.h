#pragma once

#include <string>

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
     * \brief < operator used for map container
     */
    bool operator<(const Vertex& vref) const
    {
      return id < vref.id;
    }

protected:
    const int id;
    std::string resourceType; //ToDo: Probably the type has to be changed to something else
};

}
