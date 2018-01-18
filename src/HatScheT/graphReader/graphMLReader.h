#pragma once

#include "../Graph.h"

namespace HatScheT
{
/*!
 * \brief The graphReader class use this class as base for graph reading into HatScheT
 */
class graphReader
{
public:
    /*!
     * \brief graphReader
     */
    graphReader();
    ~graphReader();
    /*!
     * \brief getReadGraph
     * \return
     */
    virtual Graph& readGraph(const char* path)=0;
private:
};

}

