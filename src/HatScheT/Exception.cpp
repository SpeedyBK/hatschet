#include <HatScheT/Exception.h>

const char* HatScheT::Exception::what() const noexcept
{
  return msg.c_str();
}

std::ostream& HatScheT::operator<<( std::ostream& oss, HatScheT::Exception &e)
{
  return oss << e.msg;
}
