#pragma once
#include <string>

namespace HatScheT
{
/*!
 * \brief The Writer class base class of all writers
 */
class Writer
{
public:
  /*!
   * \brief Writer provide a path to write to
   * \param path
   */
  Writer(std::string path);
  ~Writer();
  /*!
   * \brief write dont use the base class
   */
  virtual void write() = 0;
protected:
  /*!
   * \brief path provide a path to write to
   */
  std::string path;


};

}

