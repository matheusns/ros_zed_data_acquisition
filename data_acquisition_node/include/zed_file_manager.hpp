#ifndef ZED_FILE_MANAGER_HPP
#define ZED_FILE_MANAGER_HPP

#include <string>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

using namespace boost;
using namespace boost::filesystem;

#include <sys/statvfs.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace zed 
{
class ZedFileManager
{

public:
  ZedFileManager();

  ~ZedFileManager();

  bool isDir(std::string dir);

  bool createDir(std::string dir);

  bool existsDir(std::string dir);
  
  std::vector<std::string> listDir(std::string path);

  std::vector<std::string> listFiles(std::string path);

  std::string getUserHome();

  std::string normalizeUserPath(std::string path);

  std::string getHomeFolder();
};
}  // zed namespace
#endif