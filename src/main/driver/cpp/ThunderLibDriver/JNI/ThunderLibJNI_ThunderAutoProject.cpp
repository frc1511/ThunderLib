#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>

using namespace thunder;

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_construct
 * Signature: ()J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1construct(JNIEnv* env, jclass) {
  driver::ThunderAutoProject* project = new driver::ThunderAutoProject();
  return reinterpret_cast<jlong>(project);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_constructWithPath
 * Signature: (Ljava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1constructWithPath(
    JNIEnv* env,
    jclass,
    jstring projectPathJStr) {
  std::string projectPath = JStringToStdString(env, projectPathJStr);
  driver::ThunderAutoProject* project = new driver::ThunderAutoProject(projectPath);
  return reinterpret_cast<jlong>(project);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1delete(JNIEnv* env,
                                                                           jclass,
                                                                           jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  delete project;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_load
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1load(JNIEnv* env,
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_discoverAndLoadFromDeployDirectory
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1discoverAndLoadFromDeployDirectory(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->discoverAndLoadFromDeployDirectory();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_isLoaded
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1isLoaded(JNIEnv* env,
                                                                                 jclass,
                                                                                 jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->isLoaded();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getName(JNIEnv* env,
                                                                               jclass,
                                                                               jlong projectHandle) {
  if (!projectHandle)
    return env->NewStringUTF("");

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  std::string name = project->getName();
  return env->NewStringUTF(name.c_str());
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_hasAction
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1hasAction(JNIEnv* env,
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_isActionCommand
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1isActionCommand(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_isActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1isActionGroup(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getActionGroup
 * Signature: (JLjava/lang/String;)Ljava/util/ArrayList;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getActionGroup(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_isSequentialActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1isSequentialActionGroup(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_isConcurrentActionGroup
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1isConcurrentActionGroup(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getTrajectory
 * Signature: (JLjava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getTrajectory(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_hasTrajectory
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1hasTrajectory(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getTrajectoryNames
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getTrajectoryNames(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getAutoMode
 * Signature: (JLjava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getAutoMode(JNIEnv* env,
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_hasAutoMode
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1hasAutoMode(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getAutoModeNames
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getAutoModeNames(JNIEnv* env,
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getFieldSymmetry
 * Signature: (J)Lcom/thunder/lib/trajectory/FieldSymmetry;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getFieldSymmetry
  (JNIEnv * env, jclass, jlong projectHandle) {
  if (!projectHandle)
    return 0;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  driver::FieldSymmetry symmetry = project->getFieldSymmetry();
  return FieldSymmetryGet(env, symmetry);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_getFieldDimensions
 * Signature: (J)Lcom/thunder/lib/trajectory/FieldDimensions;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1getFieldDimensions(
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

  jobject dimensions = FieldDimensionsConstruct(env, length, width);
  return dimensions;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_setRemoteUpdatesEnabled
 * Signature: (JZ)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1setRemoteUpdatesEnabled(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_enableRemoteUpdates
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1enableRemoteUpdates(JNIEnv* env,
                                                                                        jclass,
                                                                                        jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  project->enableRemoteUpdates();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_disableRemoteUpdates
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1disableRemoteUpdates(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  project->disableRemoteUpdates();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_areRemoteUpdatesEnabled
 * Signature: (J)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1areRemoteUpdatesEnabled(
    JNIEnv* env,
    jclass,
    jlong projectHandle) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->areRemoteUpdatesEnabled();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_registerRemoteUpdateSubscriber
 * Signature: (JLjava/lang/Runnable;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1registerRemoteUpdateSubscriber(
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
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoProject_unregisterRemoteUpdateSubscriber
 * Signature: (JJ)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoProject_1unregisterRemoteUpdateSubscriber(
    JNIEnv* env,
    jclass,
    jlong projectHandle,
    jlong id) {
  if (!projectHandle)
    return false;

  driver::ThunderAutoProject* project = reinterpret_cast<driver::ThunderAutoProject*>(projectHandle);
  return project->unregisterRemoteUpdateSubscriber(static_cast<size_t>(id));
}
