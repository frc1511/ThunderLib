#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>

using namespace thunder::driver;

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1delete(JNIEnv* env, jclass, jlong modeHandle) {
  if (!modeHandle)
    return;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  delete mode;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_getFirstStep
 * Signature: (J)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1getFirstStep(JNIEnv* env,
                                                                           jclass,
                                                                           jlong modeHandle) {
  if (!modeHandle)
    return 0;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  ThunderAutoModeStep* firstStep = mode->getFirstStep();
  return reinterpret_cast<jlong>(firstStep);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_getNextStep
 * Signature: (JJ)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1getNextStep(JNIEnv* env,
                                                                          jclass,
                                                                          jlong modeHandle,
                                                                          jlong previousStepHandle) {
  if (!modeHandle || !previousStepHandle)
    return 0;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  ThunderAutoModeStep* previousStep = reinterpret_cast<ThunderAutoModeStep*>(previousStepHandle);
  ThunderAutoModeStep* nextStep = mode->getNextStep(previousStep);
  return reinterpret_cast<jlong>(nextStep);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_getFirstStepOfBoolBranch
 * Signature: (JJZ)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1getFirstStepOfBoolBranch(
    JNIEnv* env,
    jclass,
    jlong modeHandle,
    jlong branchStepHandle,
    jboolean booleanCondition) {
  if (!modeHandle || !branchStepHandle)
    return 0;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  ThunderAutoModeStep* branchStep = reinterpret_cast<ThunderAutoModeStep*>(branchStepHandle);
  ThunderAutoModeStep* firstStep =
      mode->getFirstStepOfBranch(branchStep, static_cast<bool>(booleanCondition));
  return reinterpret_cast<jlong>(firstStep);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_getFirstStepOfSwitchBranch
 * Signature: (JJI)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1getFirstStepOfSwitchBranch(
    JNIEnv* env,
    jclass,
    jlong modeHandle,
    jlong branchStepHandle,
    jint switchValue) {
  if (!modeHandle || !branchStepHandle)
    return 0;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  ThunderAutoModeStep* branchStep = reinterpret_cast<ThunderAutoModeStep*>(branchStepHandle);
  ThunderAutoModeStep* firstStep = mode->getFirstStepOfBranch(branchStep, static_cast<int>(switchValue));
  return reinterpret_cast<jlong>(firstStep);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoMode_isRunnable
 * Signature: (JJ)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode_1isRunnable(JNIEnv* env,
                                                                            jclass,
                                                                            jlong modeHandle, jlong projectHandle) {
  if (!modeHandle || !projectHandle)
    return JNI_FALSE;

  ThunderAutoMode* mode = reinterpret_cast<ThunderAutoMode*>(modeHandle);
  ThunderAutoProject* project = reinterpret_cast<ThunderAutoProject*>(projectHandle);
  bool runnable = mode->isRunnable(*project);
  return runnable ? JNI_TRUE : JNI_FALSE;
}
