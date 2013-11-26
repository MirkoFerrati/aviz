#ifndef MIRKO_PACKAGE_H
#define MIRKO_PACKAGE_H
#include <string>


namespace ros{
  namespace package{
    std::string getPath(std::string);
    std::string getPath(const char*);
    void setPath(std::string);
  }
}



#endif //MIRKO_PACKAGE_H