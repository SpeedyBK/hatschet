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
   * \brief apiTest tests API functionality
   * \return
   */
  static bool apiTest();
  /*!
   * \brief asapTest test asap scheduler
   * \return
   */
  static bool asapTest();
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
   * \brief occurrenceTest test all functions of occurrences
   * \return
   */
  static bool occurrenceTest();
  /*!
   * \brief occurrenceSetTest test all functions of occurrenceSets
   * \return
   */
  static bool occurrenceSetTest();
  /*!
   * \brief occurrenceSetCombinationTest test all functions of occurrenceSetCombinations
   * \return
   */
  static bool occurrenceSetCombinationTest();
  /*!
   * \brief sgmSchedulerTest test the subgraph modulo scheduler with a small example
   * \return
   */
  static bool sgmSchedulerTest();
  /*!
   * \brief ulSchedulerTest
   * \return
   */
  static bool ulSchedulerTest();
};
}
