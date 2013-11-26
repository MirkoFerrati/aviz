#include "package.h"
#include <assert.h>
#include <string.h>
std::string ros::package::getPath(std::string package_name)
{
assert(package_name=="rviz");
}

std::string ros::package::getPath(const char* package_name)
{
assert(strcmp(package_name,"rviz")==0);
}

void ros::package::setPath(std::string path)
{
  GLOBAL_PACKAGE_PATH==path;
}

std::string GLOBAL_PACKAGE_PATH;