#pragma once

#include <ThunderLibCore/Macros.hpp>
#include <spdlog/spdlog.h>
#include <filesystem>

namespace thunder::core {

using namespace spdlog;

// PLEASE don't have curly braces in file paths
#define LocString() "\n\tFile: " __FILE__ " \n\tLine: " TOSTRING(__LINE__)

class ThunderLibCoreLogger {
 public:
  static spdlog::logger* get();
  static void destroy();

  static void make(spdlog::sink_ptr sink);
  static void make(std::vector<spdlog::sink_ptr>::const_iterator begin,
                   std::vector<spdlog::sink_ptr>::const_iterator end);
  static void makeFileLogger(const std::filesystem::path& path, bool truncate = false);
  static void makeStdoutLogger();

#define Info(msg, ...) get()->info(msg __VA_OPT__(, ) __VA_ARGS__)
#define InfoLoc(msg, ...) get()->info(LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define Warn(msg, ...) get()->warn(msg __VA_OPT__(, ) __VA_ARGS__)
#define WarnLoc(msg, ...) get()->warn(LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define Error(msg, ...) get()->error(msg __VA_OPT__(, ) __VA_ARGS__)
#define ErrorLoc(msg, ...) get()->error(LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define Critical(msg, ...) get()->critical(msg __VA_OPT__(, ) __VA_ARGS__)
#define CriticalLoc(msg, ...) get()->critical(LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define Debug(msg, ...) get()->debug(msg __VA_OPT__(, ) __VA_ARGS__)
#define DebugLoc(msg, ...) get()->debug(LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)
};

/**
 * Returns a path to a log file in the specified logs directory.
 * The filename will be in the format: Name_YYYY-MM-DD_HH-MM-SS.log
 *
 * @param logsDirectory Directory where the log file will be created.
 * @param name Optional name prefix for the log file. If provided, it will be prepended to the filename.
 *
 * @return A path to the log file.
 */
std::filesystem::path MakeLogFilePath(const std::filesystem::path& logsDirectory,
                                      const std::string& name = "");

/**
 * Removes old log files from the specified logs directory. Logs must be
 *
 * @param logsDir
 * @param maxFiles
 */
void CleanupLogsDirectory(const std::filesystem::path& logsDir, size_t maxFiles);

}  // namespace thunder::core
