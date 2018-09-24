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
#pragma once

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>

namespace HatScheT
{
/*!
 * \brief The MoovacMinRegScheduler class
 * For the details of the Moovac minimum register scheduler, see Sittel, Kumm, Oppermann, Möller, Zipf, and Koch, ILP-based Modulo Scheduling and Binding for Register Minimization,,
 * Presented at the Proceedings of the International Conference on Field Programmable Logic and Application (FPL) 2018
 */
class MoovacMinRegScheduler : public MoovacScheduler
{
public:
    MoovacMinRegScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

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
    /*!
     * \brief getOutGoingEdgesOfResource get all edges that start from a vertex of resource r
     * \param r
     * \return
     */
    vector<Edge*> getOutGoingEdgesOfResource(Resource* r);
};
}
