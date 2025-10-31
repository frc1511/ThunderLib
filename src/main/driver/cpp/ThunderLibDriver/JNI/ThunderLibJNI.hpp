#pragma once

#include "jni.h"
#include "com_thunder_lib_jni_ThunderLibJNI.h"

#include "JavaTypes.hpp"
#include "WPILibTypes.hpp"

#include <string>

std::string JStringToStdString(JNIEnv* env, jstring jStr);

