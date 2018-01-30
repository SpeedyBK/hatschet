#pragma once
#include <string>

namespace HatScheT
{

class Writer
{
public:
  /*!
   * \brief Writer
   */
  Writer(std::string path);
  ~Writer();

  virtual void write() = 0;
protected:
  std::string path;


};

}

