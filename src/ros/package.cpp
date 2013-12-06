#include "package.h"
#include <assert.h>
#include <string.h>
// #include <iostream>
// #include <stdlib.h>
std::string GLOBAL_PACKAGE_PATH;
std::vector<std::string> ROBOT_PACKAGE_NAME;
std::vector<std::string> ROBOT_PACKAGE_PATH;

namespace ros{
namespace package{
std::string getPath(std::string package_name)
{
// assert(package_name=="rviz" || package_name==ROBOT_PACKAGE_NAME);
	if(package_name=="rviz")
		return GLOBAL_PACKAGE_PATH;
	else
		for (int i=0; i<ROBOT_PACKAGE_NAME.size(); i++)
		{
			if(package_name == ROBOT_PACKAGE_NAME[i])
			{
				return ROBOT_PACKAGE_PATH[i];
			}
		}
		
	// package not found in admissible ones
	assert(false);
}

std::string getPath(const char* package_name)
{
assert(strcmp(package_name,"rviz")==0);
return GLOBAL_PACKAGE_PATH;
}

void setPath(std::string path)
{
  GLOBAL_PACKAGE_PATH=path;
}
void getPlugins(const std::string& package, const std::string& attribute, V_string& plugins)
{
}
void getPlugins(const std::string& package, const std::string& attribute, M_string& plugins)
{
}

void setTreeVars(const std::vector<std::string>& tree_name,const std::vector<std::string>& tree_path)
{
	ROBOT_PACKAGE_NAME = tree_name;
	ROBOT_PACKAGE_PATH = tree_path;
}

}
}

