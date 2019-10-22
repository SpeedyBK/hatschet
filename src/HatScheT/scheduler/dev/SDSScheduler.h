/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
            Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

    Copyright (C) 2019

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

#ifndef HATSCHET_SDSSCHEDULER_H
#define HATSCHET_SDSSCHEDULER_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <vector>

#include "cadical.hpp"

namespace HatScheT {

  /*
   * Steve Dai, Gai Liu, Zhiru Zhang; A Scalable Approach to Exact Ressource-Constraint Scheduling Based on a Joint
   * SDC and SAT Formulation; FPGA 2018
   */

  struct bindingVariable {
    int index;
    Resource* resource;
    int resourceID;
    const Vertex* vertex;
    int resourceInstance;
    bool binding;
  };


  class SDSScheduler : public SchedulerBase, public ModuloSchedulerBase {

  public:

    /*!
		 * The SDSScheduler is a SDC and SAT based scheduler, it is based on the paper : "A Scalable Approach to Exact
     * Resource Constraint Scheduling Based on a Joint SDC and SAT Formulation" by Steve Dai, Gai Liu and Zhiru Zhang.
		 * @param g
		 * @param resourceModel
		 */
    SDSScheduler(Graph& g, ResourceModel &resourceModel);

    /*!
     * main method to do the scheduling
     */
    void schedule() override ;

    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}
    virtual void resetContainer(){/* unused */}
    virtual void constructProblem() {/* unused */}

    //Setter Functions
    /*!
     * All cout statements can be supressed by setting "quiet" to true.
     * @param quiet
     */
    void setSilent(bool quiet = true) {this -> silent = quiet;}

  private:

    //Functions
    /*!
     * This function creates Binding Variables which map each operation to an instance of the limited ressources.
     */
    void createBindingVariables();

    /*!
     * This function sets the binding variables, created by createBindingVariables, to the correct values. Such that
     * each operation is mapped to exactly one resource. It tries to map the operation equaly to the resourceinstances.
     */
    void setBindingVariables ();
    /*!
     * Creates and sets a set of boolean Sharing Variables. If the a R(ij) is true, it means that the Operations i and j
     * are sharing the same resourceinstance.
     */
    map <pair<const Vertex*, const Vertex*>, bool> createShVarsMaxSpeed();




    //Variables
    /*!
     * If true, cout statements are supressed.
     */
    bool silent;
    /*!
     * BindingType is used as a switch for the resourcebinding:
     * R -> Use minimun of resources: Starts with mapping all operations to 1 resource instance.
     * S -> Get maximum Speed : Distributes the operations as equal as possible to the resource instances.
     */
    char bindingType;
    /*!
     * Number of limited resources.
     */
    int numOfLimitedResources;
    /*!
     * Maps each operation which need a limited Resource to an instance of this ressource.
     */
    list <bindingVariable> bindingVariables;
    /*!
     * Marks if operations share the same resource instance
     */
    map <pair<const Vertex*, const Vertex*>, bool> sharingVariables;
    /*!
     * If the Ordering Variable O(i->j) is true, it means that operation i is scheduled in an earlyer clock cycle than
     * operation j
     */
    map <pair<const Vertex*, const Vertex*>, bool> orderingVariables;

  };

}

#endif //HATSCHET_SDSSCHEDULER_H
