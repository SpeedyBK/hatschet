/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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
#include <string>

namespace HatScheT
{
class Tests
{
public:
  /*!
   * \brief readTest test whether a reference graph an resource model are read correctly
   * \return
   */
  static bool readTest();
  /*!
   * test for the calculation of the critical path of a graph
   * @return
   */
  static bool cpTest();
  /*!
   * read a graph, write it, read it again, then schedule both and compare
   * @return
   */
  static bool readWriteReadScheduleTest();
  /*!
   * \brief moovacTest test whether moovac scheduling is running as expected
   * \return
   */
  static bool moovacTest();
  /*!
   * \brief moduloSDCTest test whether modulo SDC scheduling is running as expected
   * \return
   */
  static bool moduloSDCTest();
    /*!
     * \brief moduloSDCTestFiege test whether modulo SDC scheduling by nfiege is running as expected
     * \return
     */
  static bool moduloSDCTestFiege();
  /*!
   * \brief apiTest tests API functionality
   * \return
   */
  static bool apiTest();
  /*!
   * \brief asapHCTest  test asap hc scheduler
   * \return
   */
  static bool asapHCTest();
  /*!
   * \brief alapHCTest test alap hc scheduler
   * \return
   */
  static bool alapHCTest();
  /*!
   * \brief ulSchedulerTest
   * \return
   */
  static bool ulSchedulerTest();
  /*!
   * the rational min II test
   * @return
   */
  static bool rationalMinIITest();
    /*!
    * \brief Test if the Kosaraju works properly.
    * \return
    */
  static bool KosarajuTest();
    /*!
    * \brief To test the DaiZhang scheduler
    * \return
    */
  static bool DaiZhangTest();
};
}
