#include "ThunderLibJNI.hpp"
#include "com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject.h"

#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>

using namespace thunder;

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    construct
 * Signature: ()J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_construct(JNIEnv* env, jclass) {
  driver::ThunderAutoProject* project = new driver::ThunderAutoProject();
  return reinterpret_cast<jlong>(project);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    constructWithPath
 * Signature: (Ljava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_constructWithPath(
    JNIEnv* env,
    jclass,
    jstring projectPathJStr) {
  std::string projectPath = JStringToStdString(env, projectPathJStr);
  driver::ThunderAutoProject* project = new driver::ThunderAutoProject(projectPath);
  return reinterpret_cast<jlong>(project);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_delete(JNIEnv* env,
                                                                           jclass,
                                                                           jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  delete project;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    load
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_load(JNIEnv* env,
                                                                             jclass,
                                                                             jlong projectHandle,
                                                                             jstring projectPathJStr) {
  if (!projectHandle)
    return false;

  std::string projectPath = JStringToStdString(env, projectPathJStr);
  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->load(projectPath);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    discoverAndLoadFromDeployDirectory
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_discoverAndLoadFromDeployDirectory(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->discoverAndLoadFromDeployDirectory();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    isLoaded
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_isLoaded(JNIEnv* env,
                                                                                 jclass,
                                                                                 jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->isLoaded();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getName
 * Signature: (J)Ljava/lang/String;
 */
jstring Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getName(JNIEnv* env,
                                                                               jclass,
                                                                               jlong projectHandle) {
  if (!projectHandle)
    return env->NewStringUTF("");

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string name = project->getName();
  return env->NewStringUTF(name.c_str());
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    hasAction
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_hasAction(JNIEnv* env,
                                                                                  jclass,
                                                                                  jlong projectHandle,
                                                                                  jstring actionNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);
  return project->hasAction(actionName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    isActionCommand
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_isActionCommand(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring actionNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);

  std::optional<core::ThunderAutoAction> actionOptional = project->getAction(actionName);
  if (!actionOptional) {
    return false;
  }

  return actionOptional->type() == core::ThunderAutoActionType::COMMAND;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    isActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_isActionGroup(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring actionNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);

  std::optional<core::ThunderAutoAction> actionOptional = project->getAction(actionName);
  if (!actionOptional) {
    return false;
  }

  core::ThunderAutoActionType actionType = actionOptional->type();
  return actionType == core::ThunderAutoActionType::SEQUENTIAL_ACTION_GROUP ||
         actionType == core::ThunderAutoActionType::CONCURRENT_ACTION_GROUP;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getActionGroup
 * Signature: (JLjava/lang/String;)Ljava/util/ArrayList;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getActionGroup(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring actionNameJStr) {
  jobject arrayList = ArrayListConstruct(env);
  if (!projectHandle)
    return arrayList;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);

  std::optional<core::ThunderAutoAction> actionOptional = project->getAction(actionName);
  if (!actionOptional) {
    return arrayList;
  }

  std::span<const std::string> groupActionNames = actionOptional->actionGroup();
  for (const std::string& name : groupActionNames) {
    jstring nameJStr = env->NewStringUTF(name.c_str());
    ArrayListAdd(env, arrayList, nameJStr);
    env->DeleteLocalRef(nameJStr);
  }

  return arrayList;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    isSequentialActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_isSequentialActionGroup(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring actionNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);

  std::optional<core::ThunderAutoAction> actionOptional = project->getAction(actionName);
  if (!actionOptional) {
    return false;
  }

  core::ThunderAutoActionType actionType = actionOptional->type();
  return actionType == core::ThunderAutoActionType::SEQUENTIAL_ACTION_GROUP;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    isConcurrentActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_isConcurrentActionGroup(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring actionNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string actionName = JStringToStdString(env, actionNameJStr);

  std::optional<core::ThunderAutoAction> actionOptional = project->getAction(actionName);
  if (!actionOptional) {
    return false;
  }

  core::ThunderAutoActionType actionType = actionOptional->type();
  return actionType == core::ThunderAutoActionType::CONCURRENT_ACTION_GROUP;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getTrajectory
 * Signature: (JLjava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getTrajectory(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring trajectoryNameJStr) {
  if (!projectHandle)
    return 0;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string trajectoryName = JStringToStdString(env, trajectoryNameJStr);

  driver::ThunderAutoTrajectory* trajectory = project->getTrajectory(trajectoryName);
  return reinterpret_cast<jlong>(trajectory);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    hasTrajectory
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_hasTrajectory(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring trajectoryNameJstr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string trajectoryName = JStringToStdString(env, trajectoryNameJstr);
  return project->hasTrajectory(trajectoryName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getTrajectoryNames
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getTrajectoryNames(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  jobject hashSet = HashSetConstruct(env);
  if (!projectHandle)
    return hashSet;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::unordered_set<std::string> trajectoryNames = project->getTrajectoryNames();
  for (const std::string& name : trajectoryNames) {
    jstring nameJStr = env->NewStringUTF(name.c_str());
    HashSetAdd(env, hashSet, nameJStr);
    env->DeleteLocalRef(nameJStr);
  }

  return hashSet;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getAutoMode
 * Signature: (JLjava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getAutoMode(JNIEnv* env,
                                                                                 jclass,
                                                                                 jlong projectHandle,
                                                                                 jstring autoModeNameJStr) {
  if (!projectHandle)
    return 0;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string autoModeName = JStringToStdString(env, autoModeNameJStr);
  driver::ThunderAutoMode* autoMode = project->getAutoMode(autoModeName);
  return reinterpret_cast<jlong>(autoMode);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    hasAutoMode
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_hasAutoMode(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jstring autoModeNameJStr) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string autoModeName = JStringToStdString(env, autoModeNameJStr);
  return project->hasAutoMode(autoModeName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getAutoModeNames
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getAutoModeNames(JNIEnv* env,
                                                                                        jclass,
                                                                                        jlong projectHandle) {
  jobject hashSet = HashSetConstruct(env);
  if (!projectHandle)
    return hashSet;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::unordered_set<std::string> autoModeNames = project->getAutoModeNames();
  for (const std::string& name : autoModeNames) {
    jstring nameJStr = env->NewStringUTF(name.c_str());
    HashSetAdd(env, hashSet, nameJStr);
    env->DeleteLocalRef(nameJStr);
  }
  return hashSet;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getFieldSymmetry
 * Signature: (J)I
 */
jint Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getFieldSymmetry(JNIEnv* env,
                                                                                     jclass,
                                                                                     jlong projectHandle) {
  if (!projectHandle)
    return 0;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return static_cast<jint>(project->getFieldSymmetry());
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    getFieldDimensions
 * Signature: (J)Ledu/wpi/first/math/Pair;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_getFieldDimensions(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  jdouble width = 0.0, length = 0.0;

  if (projectHandle) {
    driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
    driver::FieldDimensions dimensions = project->getFieldDimensions();
    width = dimensions.width.value();
    length = dimensions.length.value();
  }

  jobject widthDouble = DoubleConstruct(env, width);
  jobject lengthDouble = DoubleConstruct(env, length);

  jobject pair = PairConstruct(env, widthDouble, lengthDouble);

  env->DeleteLocalRef(widthDouble);
  env->DeleteLocalRef(lengthDouble);

  return pair;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    setRemoteUpdatesEnabled
 * Signature: (JZ)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_setRemoteUpdatesEnabled(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jboolean enabled) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  project->setRemoteUpdatesEnabled(static_cast<bool>(enabled));
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    enableRemoteUpdates
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_enableRemoteUpdates(JNIEnv* env,
                                                                                        jclass,
                                                                                        jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  project->enableRemoteUpdates();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    disableRemoteUpdates
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_disableRemoteUpdates(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  project->disableRemoteUpdates();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    areRemoteUpdatesEnabled
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_areRemoteUpdatesEnabled(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->areRemoteUpdatesEnabled();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    registerRemoteUpdateSubscriber
 * Signature: (JLjava/lang/Runnable;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_registerRemoteUpdateSubscriber(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jobject onUpdateRunnable) {
  if (!projectHandle)
    return 0;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);

  auto callback = std::make_shared<RunnableWrapper>(env, onUpdateRunnable);
  return project->registerRemoteUpdateSubscriber([callback]() { callback->run(); });
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject
 * Method:    unregisterRemoteUpdateSubscriber
 * Signature: (JJ)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_00024ThunderAutoProject_unregisterRemoteUpdateSubscriber(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jlong id) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->unregisterRemoteUpdateSubscriber(static_cast<size_t>(id));
}
