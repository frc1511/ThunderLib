#include "ThunderLibTypes.hpp"
#include "WPILibTypes.hpp"
#include "JavaTypes.hpp"

static jclass s_thunderTrajectoryStateClass = nullptr;
static jmethodID s_thunderTrajectoryStateDefaultConstructor = nullptr;
static jmethodID s_thunderTrajectoryStateConstructor = nullptr;

static jclass s_thunderAutoFieldSymmetryClass = nullptr;
static jfieldID s_thunderAutoFieldSymmetryNoneField = nullptr;
static jfieldID s_thunderAutoFieldSymmetryRotationalField = nullptr;
static jfieldID s_thunderAutoFieldSymmetryReflectionalField = nullptr;

static jclass s_thunderAutoSendableChooserSelectionTypeClass = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeNoneField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeAutoModeField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeTrajectoryField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeCustomCommandField = nullptr;

static jclass s_thunderAutoSendableChooserSelectionClass = nullptr;
static jmethodID s_thunderAutoSendableChooserSelectionDefaultConstructor = nullptr;
static jmethodID s_thunderAutoSendableChooserSelectionConstructor = nullptr;

bool LoadThunderTrajectoryStateClass(JNIEnv* env) {
  jclass thunderTrajectoryStateClass = env->FindClass(THUNDERLIB_TRAJECTORYSTATE_SIGNATURE);
  if (!thunderTrajectoryStateClass)
    return false;

  s_thunderTrajectoryStateClass = static_cast<jclass>(env->NewGlobalRef(thunderTrajectoryStateClass));

  // new ThunderTrajectoryState()
  s_thunderTrajectoryStateDefaultConstructor =
      env->GetMethodID(s_thunderTrajectoryStateClass, "<init>", "()V");
  if (!s_thunderTrajectoryStateDefaultConstructor)
    return false;

  // new ThunderTrajectoryState(
  //     double timeSeconds,
  //     Pose2d pose2d,
  //     ChassisSpeeds chassisSpeeds,
  //     double linearVelocityMetersPerSecond,
  //     Rotation2d headingRotation2d)
  s_thunderTrajectoryStateConstructor =
      env->GetMethodID(s_thunderTrajectoryStateClass, "<init>",
                       "(DL" WPIMATH_POSE2D_SIGNATURE ";L" WPIMATH_CHASSISSPEEDS_SIGNATURE
                       ";DL" WPIMATH_ROTATION2D_SIGNATURE ";)V");
  if (!s_thunderTrajectoryStateConstructor)
    return false;

  return true;
}

void UnloadThunderTrajectoryStateClass(JNIEnv* env) {
  if (s_thunderTrajectoryStateClass) {
    env->DeleteGlobalRef(s_thunderTrajectoryStateClass);
    s_thunderTrajectoryStateClass = nullptr;
    s_thunderTrajectoryStateDefaultConstructor = nullptr;
    s_thunderTrajectoryStateConstructor = nullptr;
  }
}

jobject ThunderTrajectoryStateConstruct(JNIEnv* env) {
  jobject state = env->NewObject(s_thunderTrajectoryStateClass, s_thunderTrajectoryStateDefaultConstructor);
  return state;
}

jobject ThunderTrajectoryStateConstruct(JNIEnv* env,
                                        jdouble timeSeconds,
                                        jobject pose2d,
                                        jobject chassisSpeeds,
                                        jdouble linearVelocityMetersPerSecond,
                                        jobject headingRotation2d) {
  jobject state =
      env->NewObject(s_thunderTrajectoryStateClass, s_thunderTrajectoryStateConstructor, timeSeconds, pose2d,
                     chassisSpeeds, linearVelocityMetersPerSecond, headingRotation2d);
  return state;
}

bool LoadFieldSymmetryClass(JNIEnv* env) {
  jclass thunderAutoFieldSymmetryClass = env->FindClass(THUNDERLIB_FIELDSYMMETRY_SIGNATURE);
  if (!thunderAutoFieldSymmetryClass)
    return false;

  s_thunderAutoFieldSymmetryClass = static_cast<jclass>(env->NewGlobalRef(thunderAutoFieldSymmetryClass));

  // NONE
  s_thunderAutoFieldSymmetryNoneField = env->GetStaticFieldID(s_thunderAutoFieldSymmetryClass, "NONE",
                                                              "L" THUNDERLIB_FIELDSYMMETRY_SIGNATURE ";");
  if (!s_thunderAutoFieldSymmetryNoneField)
    return false;

  // ROTATIONAL
  s_thunderAutoFieldSymmetryRotationalField = env->GetStaticFieldID(
      s_thunderAutoFieldSymmetryClass, "ROTATIONAL", "L" THUNDERLIB_FIELDSYMMETRY_SIGNATURE ";");
  if (!s_thunderAutoFieldSymmetryRotationalField)
    return false;

  // REFLECTIONAL
  s_thunderAutoFieldSymmetryReflectionalField = env->GetStaticFieldID(
      s_thunderAutoFieldSymmetryClass, "REFLECTIONAL", "L" THUNDERLIB_FIELDSYMMETRY_SIGNATURE ";");
  if (!s_thunderAutoFieldSymmetryReflectionalField)
    return false;

  return true;
}

void UnloadFieldSymmetryClass(JNIEnv* env) {
  if (s_thunderAutoFieldSymmetryClass) {
    env->DeleteGlobalRef(s_thunderAutoFieldSymmetryClass);
    s_thunderAutoFieldSymmetryClass = nullptr;
    s_thunderAutoFieldSymmetryNoneField = nullptr;
    s_thunderAutoFieldSymmetryRotationalField = nullptr;
    s_thunderAutoFieldSymmetryReflectionalField = nullptr;
  }
}

jobject FieldSymmetryGet(JNIEnv* env, thunder::driver::FieldSymmetry symmetry) {
  switch (symmetry) {
    using enum thunder::driver::FieldSymmetry;
    case ROTATIONAL:
      return env->GetStaticObjectField(s_thunderAutoFieldSymmetryClass,
                                       s_thunderAutoFieldSymmetryRotationalField);
    case REFLECTIONAL:
      return env->GetStaticObjectField(s_thunderAutoFieldSymmetryClass,
                                       s_thunderAutoFieldSymmetryReflectionalField);
    case NONE:
    default:
      return env->GetStaticObjectField(s_thunderAutoFieldSymmetryClass, s_thunderAutoFieldSymmetryNoneField);
  }
}

bool LoadThunderAutoSendableChooser_ChooserSelection_TypeClass(JNIEnv* env) {
  jclass thunderAutoSendableChooserSelectionTypeClass =
      env->FindClass(THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE);
  if (!thunderAutoSendableChooserSelectionTypeClass)
    return false;

  s_thunderAutoSendableChooserSelectionTypeClass =
      static_cast<jclass>(env->NewGlobalRef(thunderAutoSendableChooserSelectionTypeClass));

  // NONE
  s_thunderAutoSendableChooserSelectionTypeNoneField =
      env->GetStaticFieldID(s_thunderAutoSendableChooserSelectionTypeClass, "NONE",
                            "L" THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE ";");
  if (!s_thunderAutoSendableChooserSelectionTypeNoneField)
    return false;

  // AUTO_MODE
  s_thunderAutoSendableChooserSelectionTypeAutoModeField =
      env->GetStaticFieldID(s_thunderAutoSendableChooserSelectionTypeClass, "AUTO_MODE",
                            "L" THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE ";");
  if (!s_thunderAutoSendableChooserSelectionTypeAutoModeField)
    return false;

  // TRAJECTORY
  s_thunderAutoSendableChooserSelectionTypeTrajectoryField =
      env->GetStaticFieldID(s_thunderAutoSendableChooserSelectionTypeClass, "TRAJECTORY",
                            "L" THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE ";");
  if (!s_thunderAutoSendableChooserSelectionTypeTrajectoryField)
    return false;

  // CUSTOM_COMMAND
  s_thunderAutoSendableChooserSelectionTypeCustomCommandField =
      env->GetStaticFieldID(s_thunderAutoSendableChooserSelectionTypeClass, "CUSTOM_COMMAND",
                            "L" THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE ";");
  if (!s_thunderAutoSendableChooserSelectionTypeCustomCommandField)
    return false;

  return true;
}

void UnloadThunderAutoSendableChooser_ChooserSelection_TypeClass(JNIEnv* env) {
  if (s_thunderAutoSendableChooserSelectionTypeClass) {
    env->DeleteGlobalRef(s_thunderAutoSendableChooserSelectionTypeClass);
    s_thunderAutoSendableChooserSelectionTypeClass = nullptr;
    s_thunderAutoSendableChooserSelectionTypeNoneField = nullptr;
    s_thunderAutoSendableChooserSelectionTypeAutoModeField = nullptr;
    s_thunderAutoSendableChooserSelectionTypeTrajectoryField = nullptr;
    s_thunderAutoSendableChooserSelectionTypeCustomCommandField = nullptr;
  }
}

jobject ThunderAutoSendableChooser_ChooserSelection_TypeGet(
    JNIEnv* env,
    thunder::driver::ThunderAutoSendableChooser::ChooserSelectionType type) {
  switch (type) {
    using enum thunder::driver::ThunderAutoSendableChooser::ChooserSelectionType;
    case AUTO_MODE:
      return env->GetStaticObjectField(s_thunderAutoSendableChooserSelectionTypeClass,
                                       s_thunderAutoSendableChooserSelectionTypeAutoModeField);
    case TRAJECTORY:
      return env->GetStaticObjectField(s_thunderAutoSendableChooserSelectionTypeClass,
                                       s_thunderAutoSendableChooserSelectionTypeTrajectoryField);
    case CUSTOM_COMMAND:
      return env->GetStaticObjectField(s_thunderAutoSendableChooserSelectionTypeClass,
                                       s_thunderAutoSendableChooserSelectionTypeCustomCommandField);
    case NONE:
    default:
      return env->GetStaticObjectField(s_thunderAutoSendableChooserSelectionTypeClass,
                                       s_thunderAutoSendableChooserSelectionTypeNoneField);
  }
}

bool LoadThunderAutoSendableChooser_ChooserSelectionClass(JNIEnv* env) {
  jclass thunderAutoSendableChooserSelectionClass =
      env->FindClass(THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_SIGNATURE);
  if (!thunderAutoSendableChooserSelectionClass)
    return false;

  s_thunderAutoSendableChooserSelectionClass =
      static_cast<jclass>(env->NewGlobalRef(thunderAutoSendableChooserSelectionClass));

  // new ChooserSelection()
  s_thunderAutoSendableChooserSelectionDefaultConstructor =
      env->GetMethodID(s_thunderAutoSendableChooserSelectionClass, "<init>", "()V");
  if (!s_thunderAutoSendableChooserSelectionDefaultConstructor)
    return false;

  // new ChooserSelection(Type type, String projectName, String itemName)
  s_thunderAutoSendableChooserSelectionConstructor =
      env->GetMethodID(s_thunderAutoSendableChooserSelectionClass, "<init>",
                       "(L" THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE
                       ";L" JAVA_LANG_STRING_SIGNATURE ";L" JAVA_LANG_STRING_SIGNATURE ";)V");
  if (!s_thunderAutoSendableChooserSelectionConstructor)
    return false;

  return true;
}

void UnloadThunderAutoSendableChooser_ChooserSelectionClass(JNIEnv* env) {
  if (s_thunderAutoSendableChooserSelectionClass) {
    env->DeleteGlobalRef(s_thunderAutoSendableChooserSelectionClass);
    s_thunderAutoSendableChooserSelectionClass = nullptr;
    s_thunderAutoSendableChooserSelectionDefaultConstructor = nullptr;
    s_thunderAutoSendableChooserSelectionConstructor = nullptr;
  }
}

jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(JNIEnv* env) {
  jobject selection = env->NewObject(s_thunderAutoSendableChooserSelectionClass,
                                     s_thunderAutoSendableChooserSelectionDefaultConstructor);
  return selection;
}

jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(JNIEnv* env,
                                                             jobject type,
                                                             jstring projectName,
                                                             jstring itemName) {
  jobject selection =
      env->NewObject(s_thunderAutoSendableChooserSelectionClass,
                     s_thunderAutoSendableChooserSelectionConstructor, type, projectName, itemName);
  return selection;
}
