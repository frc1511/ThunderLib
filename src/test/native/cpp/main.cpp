#include <ThunderLibTests/ThunderLibTests.hpp>
#include <fmt/format.h>
#include <ctime>

using namespace thunder;

const std::filesystem::path kTestResourcesPath =
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

  std::string outputDirName = fmt::format("ThunderLibTests_{}", timestamp);

  outputPath = kTestOutputBasePath / outputDirName;

  std::filesystem::create_directories(outputPath);

  return outputPath;
}

int main(int argc, char** argv) {
  std::filesystem::path testOutputPath = GetTestOutputPath();

  std::filesystem::path logFile = testOutputPath / "ThunderLibTests.log";
  ThunderLibLogger::makeFileLogger(logFile, true);

  ThunderLibLogger::Info("Starting ThunderLibTests...");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
