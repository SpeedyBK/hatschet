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
    MoovacMinRegScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, unsigned int minII, unsigned int maxII);
    /*!
     * \brief getNoOfImplementedRegisters return -1 if no schedule was determined
     * \return
     */
    virtual int getNoOfImplementedRegisters();
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
