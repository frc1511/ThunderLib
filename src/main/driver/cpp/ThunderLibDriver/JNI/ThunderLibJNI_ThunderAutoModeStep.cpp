#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>

using namespace thunder::driver;

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoModeStep_delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeStep_1delete(JNIEnv* env,
                                                                        jclass,
                                                                        jlong stepHandle) {
  if (!stepHandle)
    return;

  ThunderAutoModeStep* step = reinterpret_cast<ThunderAutoModeStep*>(stepHandle);
  delete step;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoModeStep_getType
 * Signature: (J)Lcom/thunder/lib/auto/ThunderAutoModeStep/Type;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeStep_1getType(JNIEnv* env,
                                                                            jclass,
                                                                            jlong stepHandle) {
  if (!stepHandle) {
    jobject unknownType = ThunderAutoModeStep_TypeGet(env, ThunderAutoModeStepType::UNKNOWN);
    return unknownType;
  }

  ThunderAutoModeStep* step = reinterpret_cast<ThunderAutoModeStep*>(stepHandle);
  ThunderAutoModeStepType type = step->getType();
  jobject typeObject = ThunderAutoModeStep_TypeGet(env, type);
  return typeObject;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoModeStep_getItemName
 * Signature: (J)Ljava/lang/String;
 */
jstring Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoModeStep_1getItemName(JNIEnv* env,
                                                                                jclass,
                                                                                jlong stepHandle) {
  if (!stepHandle) {
    return env->NewStringUTF("");
  }

  ThunderAutoModeStep* step = reinterpret_cast<ThunderAutoModeStep*>(stepHandle);
  std::string itemName = step->getItemName();
  return env->NewStringUTF(itemName.c_str());
}
