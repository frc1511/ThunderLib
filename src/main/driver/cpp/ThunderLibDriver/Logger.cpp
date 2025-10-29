#include <ThunderLibDriver/Logger.hpp>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <mutex>

#include <frc/DriverStation.h>
#include <frc/Errors.h>

static const std::string kThunderLibLoggerName = "ThunderLib";

static std::shared_ptr<spdlog::logger> s_thunderLibLogger;
static std::mutex s_loggerMutex;

namespace thunder {

static void InitLogger(const std::filesystem::path& logsDir) {
  std::lock_guard<std::mutex> lock(s_loggerMutex);

  std::vector<spdlog::sink_ptr> sinks;

  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

#ifdef __FRC_ROBORIO__
  (void)logsDir;
#else
  if (!logsDir.empty()) {
    std::filesystem::path logFile = thunder::core::MakeLogFilePath(logsDir, "ThunderLib");
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFile.string(), true));
    core::CleanupLogsDirectory(logsDir, 10);
  }
#endif

  s_thunderLibLogger = std::make_shared<spdlog::logger>(kThunderLibLoggerName, sinks.begin(), sinks.end());
  core::ThunderLibCoreLogger::make(sinks.begin(), sinks.end());  // ThunderLibCoreLogger gets the same sinks
}

spdlog::logger* ThunderLibLogger::get() {
  if (!s_thunderLibLogger) {
    InitLogger("logs");
  }

  return s_thunderLibLogger.get();
}

void ThunderLibLogger::makeFileLogger(const std::filesystem::path& path, bool truncate /*= false*/) {
  std::lock_guard<std::mutex> lock(s_loggerMutex);

  if (path.empty()) return;

  auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(), truncate);

  s_thunderLibLogger = std::make_shared<spdlog::logger>(kThunderLibLoggerName, sink);
  core::ThunderLibCoreLogger::make(sink); // ThunderLibCoreLogger gets the same sink
}

}  // namespace thunder
