#pragma once

#include <ThunderLibCore/Logger.hpp>

namespace thunder {

class ThunderLibLogger {
 public:
  static spdlog::logger* get();

  // The default logger logs to stdout and to a log file in an existing/new 'logs' directory.

  // The file logger only writes logs to the specified file.
  static void makeFileLogger(const std::filesystem::path& path, bool truncate = false);
};

}  // namespace thunder
