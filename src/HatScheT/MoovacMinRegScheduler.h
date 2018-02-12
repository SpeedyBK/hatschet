#pragma once

#include <HatScheT/MoovacScheduler.h>

namespace HatScheT
{
/*!
 * \brief The MoovacMinRegScheduler class
 */
class MoovacMinRegScheduler : public MoovacScheduler
{
public:
    MoovacMinRegScheduler(Graph& g, std::list<std::string> solverWishlist, ResourceModel &resourceModel, unsigned int minII, unsigned int maxII);
protected:
    /*!
     * \brief setGeneralConstraints read the paper for further information
     */
    virtual void setGeneralConstraints();
    /*!
     * \brief setObjective minimum register number
     */
    virtual void setObjective();
    /*!
     * \brief fillRegVector fill the container for ILP variables of registers
     */
    virtual void fillRegVector();
};
}
