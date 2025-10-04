#include "jni.h"

#if 0

#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoMode.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeStep.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeActionStep.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeTrajectoryStep.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeBoolBranchStep.h"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeSwitchBranchStep.h"

#include <ThunderLibDriver/Auto/ThunderAutoMode.h>
#include <ThunderLibDriver/Auto/ThunderAutoProject.h>

#include <cstring>

#define OBJECThunderSIGNATURE "java/lang/Object"

#define ARRAY_LISThunderSIGNATURE "java/util/ArrayList"
static jclass s_arrayListClass = nullptr;
static jmethodID s_arrayListConstructor = nullptr;
static jmethodID s_arrayListAddMethod = nullptr;

#define ROTATION_2D_SIGNATURE "edu/wpi/first/math/geometry/Rotation2d"
static jclass s_rotation2dClass = nullptr;
static jmethodID s_rotation2dConstructor = nullptr;

#define POSE_2D_SIGNATURE "edu/wpi/first/math/geometry/Pose2d"
static jclass s_pose2dClass = nullptr;
static jmethodID s_pose2dConstructor = nullptr;

static jobject createStringArrayList(JNIEnv* env,
                                     const char** strings,
                                     size_t count) {
  jobject arrayList = env->NewObject(s_arrayListClass, s_arrayListConstructor);

  for (size_t i = 0; i < count; ++i) {
    jstring str = env->NewStringUTF(strings[i]);
    env->CallBooleanMethod(arrayList, s_arrayListAddMethod, str);
    env->DeleteLocalRef(str);
  }

  return arrayList;
}

struct RunnableData {
  JNIEnv* env;
  jobject runnable;

  static RunnableData* create(JNIEnv* env, jobject runnable) {
    if (!env || !runnable)
      return nullptr;
    RunnableData* data = new RunnableData();
    data->env = env;
    data->runnable = env->NewGlobalRef(runnable);
    return data;
  }

  static void destroy(RunnableData* data) {
    if (!data)
      return;

    JNIEnv* env = data->env;
    if (env && data->runnable) {
      env->DeleteGlobalRef(data->runnable);
    }
    delete data;
  }
};

// Wraps Runnable.run() to a ThunderCallbackFunc
static void runRunable(void* runnableDataPtr) {
  RunnableData* runnableData = reinterpret_cast<RunnableData*>(runnableDataPtr);
  if (!runnableData)
    return;

  auto [env, runnable] = *runnableData;

  jclass runnableClass = env->GetObjectClass(runnable);
  jmethodID runMethod = env->GetMethodID(runnableClass, "run", "()V");

  if (runMethod) {
    env->CallVoidMethod(runnable, runMethod);
  }

  env->DeleteLocalRef(runnableClass);
}

static void cleanupCurrentRemoteUpdateCallbackData(
    ThunderAutoProject_Handle projectHandle) {
  void* runnableDataPtr =
      ThunderAutoProject_GetRemoteUpdateCallbackUserData(projectHandle);

  RunnableData* runnableData = reinterpret_cast<RunnableData*>(runnableDataPtr);
  if (!runnableData)
    return;

  auto [env, runnable] = *runnableData;

  if (env && runnable) {
    env->DeleteGlobalRef(runnable);
  }
  delete runnableData;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
  // Check to ensure the JNI version is valid

  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return JNI_ERR;

  do {
    //
    // ArrayList
    //

    jclass arrayListClass = env->FindClass(ARRAY_LISThunderSIGNATURE);
    if (!arrayListClass)
      break;
    s_arrayListClass = static_cast<jclass>(env->NewGlobalRef(arrayListClass));

    // new ArrayList<>()
    s_arrayListConstructor =
        env->GetMethodID(s_arrayListClass, "<init>", "()V");
    if (!s_arrayListConstructor)
      break;

    // boolean add(Object o)
    s_arrayListAddMethod = env->GetMethodID(s_arrayListClass, "add",
                                            "(L" OBJECThunderSIGNATURE ";)Z");
    if (!s_arrayListAddMethod)
      break;

    //
    // Rotation2d
    //

    jclass rotation2dClass = env->FindClass(ROTATION_2D_SIGNATURE);
    if (!rotation2dClass)
      break;
    s_rotation2dClass = static_cast<jclass>(env->NewGlobalRef(rotation2dClass));

    // new Rotation2d(double radians)
    s_rotation2dConstructor =
        env->GetMethodID(s_rotation2dClass, "<init>", "(D)V");
    if (!s_rotation2dConstructor)
      break;

    //
    // Pose2d
    //

    jclass pose2dClass = env->FindClass(POSE_2D_SIGNATURE);
    if (!pose2dClass)
      break;
    s_pose2dClass = static_cast<jclass>(env->NewGlobalRef(pose2dClass));

    // new Pose2d(double x, double y, Rotation2d rotation)
    s_pose2dConstructor = env->GetMethodID(s_pose2dClass, "<init>",
                                           "(DD" ROTATION_2D_SIGNATURE ";)");
    if (!s_pose2dConstructor)
      break;

    return JNI_VERSION_1_6;

  } while (0);

  return JNI_ERR;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {
  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return;

  // Unload references to ArrayList class and methods

  if (s_arrayListClass) {
    env->DeleteGlobalRef(s_arrayListClass);
    s_arrayListClass = nullptr;
    s_arrayListConstructor = nullptr;
    s_arrayListAddMethod = nullptr;
  }
}

// AutoProject

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_LoadFromFile(
    JNIEnv* env,
    jclass,
    jstring projectPathJString) {
  const char* projectPath = env->GetStringUTFChars(projectPathJString, nullptr);
  jlong projectHandle = ThunderAutoProject_LoadFromFile(projectPath);
  env->ReleaseStringUTFChars(projectPathJString, projectPath);
  return projectHandle;
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_DiscoverAndLoad(
    JNIEnv* env,
    jclass,
    jstring pathJString,
    jboolean recursive) {
  const char* path = env->GetStringUTFChars(pathJString, nullptr);
  jlong projectHandle = ThunderAutoProject_DiscoverAndLoad(path, recursive);
  env->ReleaseStringUTFChars(pathJString, path);
  return projectHandle;
}

JNIEXPORT void JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_Cleanup(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  cleanupCurrentRemoteUpdateCallbackData(projectHandle);
  ThunderAutoProject_Cleanup(projectHandle);
}

JNIEXPORT void JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_EnableRemoteUpdate(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jobject onUpdateRunnable) {
  cleanupCurrentRemoteUpdateCallbackData(projectHandle);

  RunnableData* runnableData = RunnableData::create(env, onUpdateRunnable);

  ThunderCallbackFunc onUpdate = runRunable;
  void* userData = static_cast<void*>(runnableData);
  ThunderAutoProject_EnableRemoteUpdate(projectHandle, onUpdate, userData);
}

JNIEXPORT void JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_SetRemoteUpdateCallback(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jobject onUpdateRunnable) {
  cleanupCurrentRemoteUpdateCallbackData(projectHandle);

  RunnableData* runnableData = RunnableData::create(env, onUpdateRunnable);

  ThunderCallbackFunc onUpdate = runRunable;
  void* userData = static_cast<void*>(runnableData);
  ThunderAutoProject_SetRemoteUpdateCallback(projectHandle, onUpdate, userData);
}

JNIEXPORT void JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_DisableRemoteUpdate(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  ThunderAutoProject_DisableRemoteUpdate(projectHandle);
}

JNIEXPORT jboolean JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_IsRemoteUpdateEnabled(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  return false;
}

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetPath(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  const char* path = ThunderAutoProject_GetPath(projectHandle);
  return env->NewStringUTF(path);
}

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetName(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  const char* name = ThunderAutoProject_GetName(projectHandle);
  if (!name)
    name = "";
  return env->NewStringUTF(name);
}

JNIEXPORT jobject JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetTrajectoryNames(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  size_t count = 0;
  const char** trajectoryNames =
      ThunderAutoProject_GetTrajectoryNames(projectHandle, &count);
  return createStringArrayList(env, trajectoryNames, count);
}

JNIEXPORT jobject JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetAutoModeNames(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  size_t count = 0;
  const char** autoModeNames =
      ThunderAutoProject_GetAutoModeNames(projectHandle, &count);
  return createStringArrayList(env, autoModeNames, count);
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetTrajectory(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring trajectoryNameJString) {
  const char* trajectoryName =
      env->GetStringUTFChars(trajectoryNameJString, nullptr);
  jlong trajectoryHandle = ThunderAutoProject_GetTrajectory(
      projectHandle, trajectoryName, strlen(trajectoryName));
  env->ReleaseStringUTFChars(trajectoryNameJString, trajectoryName);
  return trajectoryHandle;
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_GetAutoMode(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring autoModeNameJString) {
  const char* autoModeName =
      env->GetStringUTFChars(autoModeNameJString, nullptr);
  jlong autoModeHandle = ThunderAutoProject_GetAutoMode(
      projectHandle, autoModeName, strlen(autoModeName));
  env->ReleaseStringUTFChars(autoModeNameJString, autoModeName);
  return autoModeHandle;
}

// AutoMode

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoMode_GetName(
    JNIEnv* env,
    jclass,
    jlong autoModeHandle) {
  const char* name = ThunderAutoMode_GetName(autoModeHandle);
  if (!name)
    name = "";
  return env->NewStringUTF(name);
}

JNIEXPORT jobject JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoMode_GetInitialPose(
    JNIEnv* env,
    jclass,
    jlong autoModeHandle) {
  double x = 0.0, y = 0.0, theta = 0.0;
  ThunderAutoMode_GetInitialPose(autoModeHandle, &x, &y, &theta);

  jobject rotation2d =
      env->NewObject(s_rotation2dClass, s_rotation2dConstructor, theta);
  jobject pose2d =
      env->NewObject(s_pose2dClass, s_pose2dConstructor, x, y, rotation2d);
  return pose2d;
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoMode_GetStartStep(
    JNIEnv* env,
    jclass,
    jlong autoModeHandle) {
  return ThunderAutoMode_GetStartStep(autoModeHandle);
}

// AutoModeStep

JNIEXPORT jint JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeStep_GetType(
    JNIEnv* env,
    jclass,
    jlong stepHandle) {
  return ThunderAutoModeStep_GetType(stepHandle);
}

// AutoModeActionStep

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeActionStep_GetActionName(
    JNIEnv* env,
    jclass,
    jlong actionStepHandle) {
  const char* actionName =
      ThunderAutoModeActionStep_GetActionName(actionStepHandle);
  if (!actionName)
    actionName = "";
  return env->NewStringUTF(actionName);
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeActionStep_GetNextStep(
    JNIEnv* env,
    jclass,
    jlong actionStepHandle) {
  return ThunderAutoModeStep_GetNextStep(actionStepHandle);
}

// AutoModeTrajectoryStep

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeTrajectoryStep_GetTrajectoryName(
    JNIEnv* env,
    jclass,
    jlong trajectoryStepHandle) {
  const char* trajectoryName =
      ThunderAutoModeTrajectoryStep_GetTrajectoryName(trajectoryStepHandle);
  if (!trajectoryName)
    trajectoryName = "";
  return env->NewStringUTF(trajectoryName);
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeTrajectoryStep_GetNextStep(
    JNIEnv* env,
    jclass,
    jlong trajectoryStepHandle) {
  return ThunderAutoModeStep_GetNextStep(trajectoryStepHandle);
}

// AutoModeBoolBranchStep

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeBoolBranchStep_GetConditionName(
    JNIEnv* env,
    jclass,
    jlong branchStepHandle) {
  const char* conditionName =
      ThunderAutoModeBoolBranchStep_GetConditionName(branchStepHandle);
  if (!conditionName)
    conditionName = "";
  return env->NewStringUTF(conditionName);
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeBoolBranchStep_GetNextStep(
    JNIEnv* env,
    jclass,
    jlong branchStepHandle,
    jboolean conditionResult) {
  return ThunderAutoModeBoolBranchStep_GetNextStep(branchStepHandle,
                                                   conditionResult);
}

// AutoModeSwitchBranchStep

JNIEXPORT jstring JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeSwitchBranchStep_GetConditionName(
    JNIEnv* env,
    jclass,
    jlong branchStepHandle) {
  const char* conditionName =
      ThunderAutoModeSwitchBranchStep_GetConditionName(branchStepHandle);
  if (!conditionName)
    conditionName = "";
  return env->NewStringUTF(conditionName);
}

JNIEXPORT jlong JNICALL
Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoModeSwitchBranchStep_GetNextStep(
    JNIEnv* env,
    jclass,
    jlong branchStepHandle,
    jint conditionResult) {
  return ThunderAutoModeSwitchBranchStep_GetNextStep(branchStepHandle,
                                                     conditionResult);
}

#endif
