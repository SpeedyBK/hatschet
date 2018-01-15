#include <HatScheT/Exception.h>

namespace HatScheT
{

const char* Exception::what() const noexcept
{
  return msg.c_str();
}

std::ostream& operator<<( std::ostream& oss, HatScheT::Exception &e)
{
  return oss << e.msg;
}

}
