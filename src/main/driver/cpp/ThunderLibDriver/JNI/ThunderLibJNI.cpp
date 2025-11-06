#include "ThunderLibJNI.hpp"

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
    if (!LoadIntegerClass(env))
      break;
    if (!LoadDoubleClass(env))
      break;
    if (!LoadArrayListClass(env))
      break;
    if (!LoadHashSetClass(env))
      break;
    if (!LoadHashMapClass(env))
      break;

    if (!LoadRotation2dClass(env))
      break;
    if (!LoadPose2dClass(env))
      break;
    if (!LoadChassisSpeedsClass(env))
      break;
    if (!LoadPairClass(env))
      break;

    if (!LoadThunderTrajectoryStateClass(env))
      break;
    if (!LoadFieldSymmetryClass(env))
      break;
    if (!LoadFieldDimensionsClass(env))
      break;
    if (!LoadThunderAutoSendableChooser_ChooserSelection_TypeClass(env))
      break;

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
}
