#pragma once

#include <ThunderLibCore/Logger.hpp>
#include <ThunderLibCore/Macros.hpp>
#include <ThunderLibCore/Error.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <filesystem>

// Path where resource files for tests are located
extern const std::filesystem::path kTestDataPath;

/**
 * Path to store output files from tests (logs, CSV exports, etc.).
 * This path is generated for the current test run.
 */
std::filesystem::path GetTestOutputPath();

