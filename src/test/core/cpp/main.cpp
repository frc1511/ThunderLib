#include <ThunderLibCoreTests/ThunderLibCoreTests.hpp>
#include <fmt/format.h>
#include <ctime>

using namespace thunder::core;

const std::filesystem::path kTestDataPath =
    std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "resources";

static const std::filesystem::path kTestOutputBasePath = std::filesystem::current_path() / "TestOutput";

std::filesystem::path GetTestOutputPath() {
  static std::filesystem::path outputPath;
  static bool initialized = false;

  if (initialized) {
    return outputPath;
  }

  time_t now = std::time(nullptr);
  std::tm* localTime = std::localtime(&now);

  std::string timestamp =
      fmt::format("{:04}-{:02}-{:02}_{:02}-{:02}-{:02}", localTime->tm_year + 1900, localTime->tm_mon + 1,
                  localTime->tm_mday, localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

  std::string outputDirName = fmt::format("ThunderLibCoreTests_{}", timestamp);

  outputPath = kTestOutputBasePath / outputDirName;

  std::filesystem::create_directories(outputPath);

  return outputPath;
}

int main(int argc, char** argv) {
  std::filesystem::path testOutputPath = GetTestOutputPath();

  std::filesystem::path logFile = testOutputPath / "ThunderLibCoreTests.log";
  ThunderLibCoreLogger::makeFileLogger(logFile, true);

  ThunderLibCoreLogger::Info("Starting ThunderLibCoreTests...");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
