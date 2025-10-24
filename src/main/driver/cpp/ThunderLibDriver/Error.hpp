#pragma once

#include "Logger.hpp"
#include <ThunderLibCore/Error.hpp>

#define ThunderLibAssert(condition, ...) TAssert(ThunderLibLogger, condition __VA_OPT__(, ) __VA_ARGS__)
#define ThunderLibUnreachable(...) TUnreachable(ThunderLibLogger __VA_OPT__(, ) __VA_ARGS__)
