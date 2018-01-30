#pragma once
#include "Writer.h"

namespace HatScheT
{

class DotWriter : public Writer
{
public:
  /*!
   * \brief Writer
   */
  DotWriter(std::string path);
  ~DotWriter();

  virtual void write();

protected:


};

}

