#include "package.h"
#include <assert.h>
#include <string.h>
std::string GLOBAL_PACKAGE_PATH;

namespace ros{
namespace package{
std::string getPath(std::string package_name)
{
assert(package_name=="rviz");
}

std::string getPath(const char* package_name)
{
assert(strcmp(package_name,"rviz")==0);
}

void setPath(std::string path)
{
  GLOBAL_PACKAGE_PATH==path;
}
void getPlugins(const std::string& package, const std::string& attribute, V_string& plugins)
{
}
void getPlugins(const std::string& package, const std::string& attribute, M_string& plugins)
{
}

}
}

