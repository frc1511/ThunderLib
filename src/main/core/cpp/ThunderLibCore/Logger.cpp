#include <ThunderLibCore/Logger.hpp>
#include <ThunderLibCore/Error.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <fmt/format.h>
#include <mutex>
#include <ctime>
#include <string>

namespace thunder::core {

static const std::string kThunderLibCoreLoggerName = "ThunderLibCore";

static std::mutex s_loggerMutex;
static std::shared_ptr<spdlog::logger> s_thunderLibLogger;

spdlog::logger* ThunderLibCoreLogger::get() {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  if (!s_thunderLibLogger) {
    s_thunderLibLogger = spdlog::stdout_color_mt(kThunderLibCoreLoggerName);
  }
  return s_thunderLibLogger.get();
}

void ThunderLibCoreLogger::destroy() {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  drop(kThunderLibCoreLoggerName);
  s_thunderLibLogger.reset();
}

void ThunderLibCoreLogger::make(spdlog::sink_ptr sink) {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  drop(kThunderLibCoreLoggerName);
  s_thunderLibLogger.reset();
  s_thunderLibLogger = std::make_shared<spdlog::logger>(kThunderLibCoreLoggerName, std::move(sink));
}

void ThunderLibCoreLogger::make(std::vector<spdlog::sink_ptr>::const_iterator begin,
                                std::vector<spdlog::sink_ptr>::const_iterator end) {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  drop(kThunderLibCoreLoggerName);
  s_thunderLibLogger.reset();
  s_thunderLibLogger = std::make_shared<spdlog::logger>(kThunderLibCoreLoggerName, begin, end);
}

void ThunderLibCoreLogger::makeFileLogger(const std::filesystem::path& path, bool truncate) {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  drop(kThunderLibCoreLoggerName);
  s_thunderLibLogger.reset();
  if (path.empty()) {
    throw InvalidArgumentError::Construct("Log file path cannot be empty");
  }
  s_thunderLibLogger = spdlog::basic_logger_mt(kThunderLibCoreLoggerName, path.string(), truncate);
}

void ThunderLibCoreLogger::makeStdoutLogger() {
  std::lock_guard<std::mutex> lock(s_loggerMutex);
  drop(kThunderLibCoreLoggerName);
  s_thunderLibLogger.reset();
  s_thunderLibLogger = spdlog::stdout_color_mt(kThunderLibCoreLoggerName);
}

std::filesystem::path MakeLogFilePath(const std::filesystem::path& logsDirectory,
                                      const std::string& name /*= ""*/) {
  time_t now = std::time(nullptr);
  std::tm* localTime = std::localtime(&now);

  std::string timestamp =
      fmt::format("{:04}-{:02}-{:02}_{:02}-{:02}-{:02}", localTime->tm_year + 1900, localTime->tm_mon + 1,
                  localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

  std::string filename;
  if (!name.empty()) {
    filename = name + '_';
  }
  filename += timestamp + ".log";

  return logsDirectory / filename;
}

struct LogFileTimestamp {
  int year, month, day, hour, minute, second;

  bool operator<(const LogFileTimestamp& other) const {
    if (year != other.year)
      return year < other.year;

    if (month != other.month)
      return month < other.month;

    if (day != other.day)
      return day < other.day;

    if (hour != other.hour)
      return hour < other.hour;

    if (minute != other.minute)
      return minute < other.minute;

    if (second != other.second)
      return second < other.second;

    return false;
  }
};

struct LogFile {
  LogFileTimestamp timestamp;
  std::filesystem::path path;

  bool operator<(const LogFile& other) const { return timestamp < other.timestamp; }
};

void CleanupLogsDirectory(const std::filesystem::path& logsDir, size_t maxFiles) {
  if (!std::filesystem::exists(logsDir)) {
    ThunderLibCoreLogger::ErrorLoc("Logs dir `{}' does not exist", logsDir.string());
    return;
  }

  std::vector<LogFile> logFiles;

  for (auto const& dirEntry : std::filesystem::directory_iterator(logsDir)) {
    if (!dirEntry.is_regular_file())
      continue;

    std::filesystem::path path = dirEntry.path();

    if (path.extension() != ".log")
      continue;

    std::string stem = path.stem().string();

    /**
     * Make sure the name fits the Name_YYYY-MM-DD_HH-MM-SS.log format. If it runs into an error at any point,
     * just skip the file.
     */
    if (stem.size() < 19)
      continue;

    std::string_view timestampString = stem;

    if (stem.size() > 19) {
      timestampString.remove_prefix(stem.size() - 20);

      if (timestampString.front() != '_')
        continue;

      timestampString.remove_prefix(1);
    }

    LogFileTimestamp timestamp;

    try {
      int year = std::stoi(std::string(timestampString.substr(0, 4)));
      timestampString.remove_prefix(5);

      int month = std::stoi(std::string(timestampString.substr(0, 2)));
      timestampString.remove_prefix(3);

      int day = std::stoi(std::string(timestampString.substr(0, 2)));
      timestampString.remove_prefix(3);

      int hour = std::stoi(std::string(timestampString.substr(0, 2)));
      timestampString.remove_prefix(3);

      int minute = std::stoi(std::string(timestampString.substr(0, 2)));
      timestampString.remove_prefix(3);

      int second = std::stoi(std::string(timestampString));

      timestamp = LogFileTimestamp{
          .year = year, .month = month, .day = day, .hour = hour, .minute = minute, .second = second};
    }

    catch (...) {
      continue;
    }

    logFiles.push_back(LogFile{timestamp, path});
  }

  std::sort(logFiles.begin(), logFiles.end());

  if (logFiles.size() > maxFiles) {
    for (size_t i = 0; i < logFiles.size() - maxFiles; i++) {
      std::filesystem::remove(logFiles.at(i).path);
    }
  }
}

}  // namespace thunder::core
