#ifndef MIRKO_PACKAGE_H
#define MIRKO_PACKAGE_H
#include <string>
#include <vector>
#include <map>

namespace ros{
  namespace package{
    typedef std::vector<std::string> V_string;
    typedef std::map<std::string, std::string> M_string;
    std::string getPath(std::string);
    std::string getPath(const char*);
    void setPath(std::string);
    void getPlugins(const std::string& package, const std::string& attribute, M_string& plugins);
    void getPlugins(const std::string& package, const std::string& attribute, V_string& plugins);
  }
}

extern std::string GLOBAL_PACKAGE_PATH;

#endif //MIRKO_PACKAGE_H
