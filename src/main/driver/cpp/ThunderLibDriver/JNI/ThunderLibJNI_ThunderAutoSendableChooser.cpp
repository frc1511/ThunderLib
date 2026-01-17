#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Auto/ThunderAutoSendableChooser.hpp>

using namespace thunder::driver;

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_construct
 * Signature: (Ljava/util/function/Consumer;Ljava/util/function/Consumer;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1construct(
    JNIEnv* env,
    jclass,
    jobject addChooserSelectionConsumer,
    jobject publishChooserConsumer) {
  auto rawAddChooserSelection = std::make_shared<ConsumerWrapper>(env, addChooserSelectionConsumer);
  auto rawPublishChooser = std::make_shared<ConsumerWrapper>(env, publishChooserConsumer);

  auto addChooserSelection = [rawAddChooserSelection](const ThunderAutoSendableChooserSelection& selection) {
    JNIEnv* env = rawAddChooserSelection->getEnv();
    jobject jSelection = ThunderAutoSendableChooser_ChooserSelectionConstruct(env, selection);
    rawAddChooserSelection->accept(jSelection);
    env->DeleteLocalRef(jSelection);
  };

  auto publishChooser = [rawPublishChooser](const std::string& smartDashboardKey) {
    JNIEnv* env = rawPublishChooser->getEnv();
    jstring jSmartDashboardKey = env->NewStringUTF(smartDashboardKey.c_str());
    rawPublishChooser->accept(jSmartDashboardKey);
    env->DeleteLocalRef(jSmartDashboardKey);
  };

  ThunderAutoSendableChooser* chooser = new ThunderAutoSendableChooser(addChooserSelection, publishChooser);
  return reinterpret_cast<jlong>(chooser);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_constructWithSmartDashboardKey
 * Signature: (Ljava/util/function/Consumer;Ljava/util/function/Consumer;Ljava/lang/String;)J
 */
jlong Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1constructWithSmartDashboardKey(
    JNIEnv* env,
    jclass,
    jobject addChooserSelectionConsumer,
    jobject publishChooserConsumer,
    jstring smartDashboardKeyJStr) {
  auto rawAddChooserSelection = std::make_shared<ConsumerWrapper>(env, addChooserSelectionConsumer);
  auto rawPublishChooser = std::make_shared<ConsumerWrapper>(env, publishChooserConsumer);

  auto addChooserSelection = [rawAddChooserSelection](const ThunderAutoSendableChooserSelection& selection) {
    JNIEnv* env = rawAddChooserSelection->getEnv();
    jobject jSelection = ThunderAutoSendableChooser_ChooserSelectionConstruct(env, selection);
    rawAddChooserSelection->accept(jSelection);
    env->DeleteLocalRef(jSelection);
  };

  auto publishChooser = [rawPublishChooser](const std::string& smartDashboardKey) {
    JNIEnv* env = rawPublishChooser->getEnv();
    jstring jSmartDashboardKey = env->NewStringUTF(smartDashboardKey.c_str());
    rawPublishChooser->accept(jSmartDashboardKey);
    env->DeleteLocalRef(jSmartDashboardKey);
  };

  std::string smartDashboardKey = JStringToStdString(env, smartDashboardKeyJStr);
  ThunderAutoSendableChooser* chooser =
      new ThunderAutoSendableChooser(addChooserSelection, publishChooser, smartDashboardKey);
  return reinterpret_cast<jlong>(chooser);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1delete(JNIEnv* env,
                                                                               jclass,
                                                                               jlong chooserHandle) {
  if (!chooserHandle)
    return;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  delete chooser;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_publish
 * Signature: (JLjava/lang/String;)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1publish(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring smartDashboardKeyJStr) {
  if (!chooserHandle)
    return;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string smartDashboardKey = JStringToStdString(env, smartDashboardKeyJStr);
  chooser->publish(smartDashboardKey);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_includeProjectSource
 * Signature: (JJZZ)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1includeProjectSource(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jlong projectHandle,
    jboolean addAllAutoModes,
    jboolean addAllTrajectories) {
  if (!chooserHandle || !projectHandle)
    return;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  ThunderAutoProject* project = reinterpret_cast<ThunderAutoProject*>(projectHandle);
  chooser->includeProjectSource(project, addAllAutoModes, addAllTrajectories);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_addAllTrajectoriesFromProject
 * Signature: (JLjava/lang/String;)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1addAllTrajectoriesFromProject(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring projectNameJStr) {
  if (!chooserHandle)
    return;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string projectName = JStringToStdString(env, projectNameJStr);
  chooser->addAllTrajectoriesFromProject(projectName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_addAllAutoModesFromProject
 * Signature: (JLjava/lang/String;)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1addAllAutoModesFromProject(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring projectNameJStr) {
  if (!chooserHandle)
    return;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string projectName = JStringToStdString(env, projectNameJStr);
  chooser->addAllAutoModesFromProject(projectName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_addTrajectoryFromProject
 * Signature: (JLjava/lang/String;Ljava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1addTrajectoryFromProject(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring projectNameJStr,
    jstring trajectoryNameJStr) {
  if (!chooserHandle)
    return false;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string projectNameStr = JStringToStdString(env, projectNameJStr);
  std::string trajectoryNameStr = JStringToStdString(env, trajectoryNameJStr);
  return chooser->addTrajectoryFromProject(projectNameStr, trajectoryNameStr);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_addCustomCommand
 * Signature: (JLjava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1addCustomCommand(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring commandNameJStr) {
  if (!chooserHandle)
    return false;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string commandName = JStringToStdString(env, commandNameJStr);
  return chooser->addCustomCommand(commandName);
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoSendableChooser_addAutoModeFromProject
 * Signature: (JLjava/lang/String;Ljava/lang/String;)Z
 */
jboolean Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoSendableChooser_1addAutoModeFromProject(
    JNIEnv* env,
    jclass,
    jlong chooserHandle,
    jstring projectNameJStr,
    jstring autoModeNameJStr) {
  if (!chooserHandle)
    return false;

  ThunderAutoSendableChooser* chooser = reinterpret_cast<ThunderAutoSendableChooser*>(chooserHandle);
  std::string projectName = JStringToStdString(env, projectNameJStr);
  std::string autoModeName = JStringToStdString(env, autoModeNameJStr);
  return chooser->addAutoModeFromProject(projectName, autoModeName);
}
