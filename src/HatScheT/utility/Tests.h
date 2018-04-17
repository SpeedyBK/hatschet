/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
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
   * \brief moovacTest test whether moovac scheduling is runninv as expected
   * \return
   */
  static bool moovacTest();
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
   * \brief asapHCTest
   * \return
   */
  static bool asapHCTest();
  /*!
   * \brief alapHCTest
   * \return
   */
  static bool alapHCTest();
  /*!
   * \brief occurrenceTest
   * \return
   */
  static bool occurrenceTest();
  /*!
   * \brief occurrenceSetTest
   * \return
   */
  static bool occurrenceSetTest();
  /*!
   * \brief occurrenceSetCombinationTest
   * \return
   */
  static bool occurrenceSetCombinationTest();
  /*!
   * \brief sgmSchedulerTest
   * \return
   */
  static bool sgmSchedulerTest();
};
}
