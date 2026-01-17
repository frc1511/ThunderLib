#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Logger.hpp>

using namespace thunder;

std::string JStringToStdString(JNIEnv* env, jstring jStr) {
  if (!jStr)
    return "";

  const char* chars = env->GetStringUTFChars(jStr, nullptr);
  std::string str(chars);
  env->ReleaseStringUTFChars(jStr, chars);
  return str;
}

jint JNI_OnLoad(JavaVM* vm, void* reserved) {
  // Check to ensure the JNI version is valid

  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return JNI_ERR;

  // Load all required classes

  do {
    // Java types

    if (!LoadIntegerClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load Integer class");
      break;
    }
    if (!LoadDoubleClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load Double class");
      break;
    }
    if (!LoadArrayListClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load ArrayList class");
      break;
    }
    if (!LoadHashSetClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load HashSet class");
      break;
    }
    if (!LoadHashMapClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load HashMap class");
      break;
    }

    // WPILib types

    if (!LoadRotation2dClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load Rotation2d class");
      break;
    }
    if (!LoadPose2dClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load Pose2d class");
      break;
    }
    if (!LoadChassisSpeedsClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load ChassisSpeeds class");
      break;
    }
    if (!LoadPairClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load Pair class");
      break;
    }

    // ThunderLib types

    if (!LoadThunderTrajectoryStateClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load ThunderTrajectoryState class");
      break;
    }
    if (!LoadFieldSymmetryClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load FieldSymmetry class");
      break;
    }
    if (!LoadFieldDimensionsClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load FieldDimensions class");
      break;
    }
    if (!LoadThunderAutoSendableChooser_ChooserSelection_TypeClass(env)) {
      ThunderLibLogger::Error(
          "[JNI_OnLoad] Failed to load ThunderAutoSendableChooser_ChooserSelection_Type class");
      break;
    }
    if (!LoadThunderAutoSendableChooser_ChooserSelectionClass(env)) {
      ThunderLibLogger::Error(
          "[JNI_OnLoad] Failed to load ThunderAutoSendableChooser_ChooserSelection class");
      break;
    }
    if (!LoadThunderAutoModeStep_TypeClass(env)) {
      ThunderLibLogger::Error("[JNI_OnLoad] Failed to load ThunderAutoModeStep_Type class");
      break;
    }

    return JNI_VERSION_1_6;
  } while (0);

  return JNI_ERR;
}

void JNI_OnUnload(JavaVM* vm, void* reserved) {
  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return;

  UnloadIntegerClass(env);
  UnloadDoubleClass(env);
  UnloadArrayListClass(env);
  UnloadHashSetClass(env);
  UnloadHashMapClass(env);

  UnloadRotation2dClass(env);
  UnloadPose2dClass(env);
  UnloadChassisSpeedsClass(env);
  UnloadPairClass(env);

  UnloadThunderTrajectoryStateClass(env);
  UnloadFieldSymmetryClass(env);
  UnloadFieldDimensionsClass(env);
  UnloadThunderAutoSendableChooser_ChooserSelection_TypeClass(env);
  UnloadThunderAutoSendableChooser_ChooserSelectionClass(env);
  UnloadThunderAutoModeStep_TypeClass(env);
}
