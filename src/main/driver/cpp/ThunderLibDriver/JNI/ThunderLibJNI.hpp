#pragma once

#include "jni.h"

#include "JavaTypes.hpp"
#include "WPILibTypes.hpp"

#include <string>

std::string JStringToStdString(JNIEnv* env, jstring jStr);

