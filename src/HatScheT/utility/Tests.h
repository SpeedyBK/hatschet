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

};
}
