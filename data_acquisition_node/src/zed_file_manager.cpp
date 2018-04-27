#include <include/zed_file_manager.hpp>

using namespace boost;
using namespace boost::filesystem;

namespace zed
{
ZedFileManager::ZedFileManager() {}

ZedFileManager::~ZedFileManager()
{
  
}

std::string ZedFileManager::getUserHome()
{
  std::string path = "~/";
  const char *home = getenv("HOME");
  if (home == 0)
  {
    path = normalizeUserPath(path);
    return path;
  }
  else
  {
    return home;
  }
}

std::string ZedFileManager::normalizeUserPath(std::string path)
{
  if (path[0] == '~')
  {
    char *home = getenv("HOME");
    return home + path.substr(1);
  }
  else
  {
    return path;
  }
}


std::vector<std::string> ZedFileManager::listDir(std::string path)
{
  path = normalizeUserPath(path);

  std::vector<std::string> list;
  try
  {
    if (!exists(path))
    {
      return list;
    }

    directory_iterator end_itr;

    for (directory_iterator itr(path); itr != end_itr; ++itr)
    {
      if (is_directory(itr->path()))
      {
        list.push_back(itr->path().c_str());
      }
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  return list;
}

bool ZedFileManager::isDir(std::string dir)
{

  boost::filesystem::path p(normalizeUserPath(dir));

  bool result;

  try
  {
    file_status s = status(p);
    result = is_directory(s);
    return result;
  }
  catch (filesystem_error &e)
  {
    std::cerr << e.what() << '\n';
  }
  return false;
}

bool ZedFileManager::createDir(std::string dir)
{

  boost::filesystem::path p(normalizeUserPath(dir));

  try
  {
    if (!exists(p))
    {
      if (!filesystem::create_directories(p))
      {
        return true;
      }
    }
  }
  catch (filesystem_error &e)
  {
    std::cerr << e.what() << '\n';
  }
  return false;
}

std::vector<std::string> ZedFileManager::listFiles(std::string path)
{
  path = normalizeUserPath(path);
  std::vector<std::string> list;

  try
  {
    if (!exists(path))
    {
      return list;
    }

    directory_iterator end_itr;

    for (directory_iterator itr(path); itr != end_itr; ++itr)
    {
      if (!is_directory(itr->path()))
      {
        list.push_back(itr->path().c_str());
      }
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  return list;
}

bool ZedFileManager::existsDir(std::string dir_path) 
{
  filesystem::path config_folder(dir_path);

  if ( !(filesystem::exists(config_folder)) ) return false;
  return true;
}

std::string ZedFileManager::getHomeFolder()
{
  const char *homeDir;

  if ((homeDir = getenv("HOME")) == NULL)
  {
    homeDir = getpwuid(getuid())->pw_dir;
  }

  return std::string(homeDir);
}
}