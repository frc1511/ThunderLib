#pragma once

#include <ThunderLibCore/Logger.hpp>
#include <ThunderLibCore/Concepts.hpp>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>
#include <fmt/format.h>

namespace thunder::core {

#if THUNDERLIB_WITH_ASSERTIONS
#define TAssert(Logger, condition, ...)                                       \
  do {                                                                        \
    if (!(condition)) {                                                       \
      Logger::CriticalLoc("Assertion `" #condition "' failed. " __VA_ARGS__); \
      std::abort();                                                           \
    }                                                                         \
  } while (false)
#else
#define TAssert(...)
#endif

#define TUnreachable(Logger, ...)                                  \
  do {                                                             \
    Logger::CriticalLoc("Unreachable code reached. " __VA_ARGS__); \
    std::abort();                                                  \
  } while (false)

#define ThunderLibCoreAssert(condition, ...) \
  TAssert(ThunderLibCoreLogger, condition __VA_OPT__(, ) __VA_ARGS__)

#define ThunderLibCoreUnreachable(...) TUnreachable(ThunderLibCoreLogger __VA_OPT__(, ) __VA_ARGS__)

class ThunderError : public std::exception {
  const char* m_file;
  unsigned long m_line;
  std::string m_message;
  std::string m_what;

 public:
  ThunderError(const char* file, unsigned long line, const std::string& message)
      : m_file(file),
        m_line(line),
        m_message(message),
        m_what(fmt::format("\n\tFile: {}\n\tLine: {}\n\tMessage: {}", file, line, message)) {}

  static ThunderError ConstructEx(const char* file, unsigned long line, const std::string& message) {
    return ThunderError(file, line, message);
  }

#define Construct(message, ...) \
  ConstructEx(__FILE__, __LINE__, fmt::format(message __VA_OPT__(, ) __VA_ARGS__))

  const char* what() const noexcept override { return m_what.c_str(); }

  const char* file() const { return m_file; }
  unsigned long line() const { return m_line; }
  std::string_view message() const { return m_message; }
};

class LogicError : public ThunderError {};
class InvalidArgumentError : public ThunderError {};
class DomainError : public ThunderError {};
class LengthError : public ThunderError {};
class OutOfRangeError : public ThunderError {};

class RuntimeError : public ThunderError {};

}  // namespace thunder::core
