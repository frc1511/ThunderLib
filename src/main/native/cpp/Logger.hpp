#pragma once

#include <ThunderLibCore/Logger.hpp>

namespace thunder {

class ThunderLibLogger {
 public:
  static spdlog::logger* get();
};

}  // namespace thunder
