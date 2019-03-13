/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)

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
#include <HatScheT/utility/Exception.h>

namespace HatScheT {
/*!
 * is the base class of all schedulers that solve the scheduling problem iteratively.
 */
  class IterativeSchedulerBase {
  public:
    IterativeSchedulerBase() {
      //default infinity
      this->maxRuns = -1;
      this->timeouts = 0;
    };

    ~IterativeSchedulerBase() {};

    /*!
     * this values can be used to limit the maximum number of runs
     * for solving the scheduling problem iteratively
     * @param m
     */
    void setMaxRuns(int m);
    int getMaxRuns() { return this->maxRuns; }
    /*!
     * return the timeouts that may have occurred
     * @return
     */
    unsigned int getTimeouts(){return this->timeouts;}
  protected:
    /*!
     * this values can be used to limit the maximum number of runs
     * for solving the scheduling problem iteratively
     */
    int maxRuns;
    /*!
   * count the timeouts that may have occurred
   */
    unsigned int timeouts;
};
}
