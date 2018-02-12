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

};
}
