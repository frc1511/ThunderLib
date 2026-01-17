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

static jclass s_thudnerAutoFieldDimensionsClass = nullptr;
static jmethodID s_thunderAutoFieldDimensionsConstructor = nullptr;

static jclass s_thunderAutoSendableChooserSelectionTypeClass = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeNoneField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeAutoModeField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeTrajectoryField = nullptr;
static jfieldID s_thunderAutoSendableChooserSelectionTypeCustomCommandField = nullptr;

static jclass s_thunderAutoSendableChooserSelectionClass = nullptr;
static jmethodID s_thunderAutoSendableChooserSelectionDefaultConstructor = nullptr;
static jmethodID s_thunderAutoSendableChooserSelectionConstructor = nullptr;

static jclass s_thunderAutoModeStepTypeClass = nullptr;
static jfieldID s_thunderAutoModeStepTypeUnknownField = nullptr;
static jfieldID s_thunderAutoModeStepTypeActionField = nullptr;
static jfieldID s_thunderAutoModeStepTypeTrajectoryField = nullptr;
static jfieldID s_thunderAutoModeStepTypeBranchBoolField = nullptr;
static jfieldID s_thunderAutoModeStepTypeBranchSwitchField = nullptr;

bool LoadThunderTrajectoryStateClass(JNIEnv* env) {
  jclass thunderTrajectoryStateClass = env->FindClass(THUNDERLIB_TRAJECTORYSTATE_SIGNATURE);
  if (!thunderTrajectoryStateClass)
    return false;

  s_thunderTrajectoryStateClass = static_cast<jclass>(env->NewGlobalRef(thunderTrajectoryStateClass));

  env->DeleteLocalRef(thunderTrajectoryStateClass);

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

  env->DeleteLocalRef(thunderAutoFieldSymmetryClass);

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

bool LoadFieldDimensionsClass(JNIEnv* env) {
  jclass thunderAutoFieldDimensionsClass = env->FindClass(THUNDERLIB_FIELDDIMENSIONS_SIGNATURE);
  if (!thunderAutoFieldDimensionsClass)
    return false;

  s_thudnerAutoFieldDimensionsClass = static_cast<jclass>(env->NewGlobalRef(thunderAutoFieldDimensionsClass));

  env->DeleteLocalRef(thunderAutoFieldDimensionsClass);

  // new FieldDimensions(double lengthMeters, double widthMeters)
  s_thunderAutoFieldDimensionsConstructor =
      env->GetMethodID(s_thudnerAutoFieldDimensionsClass, "<init>", "(DD)V");
  if (!s_thunderAutoFieldDimensionsConstructor)
    return false;

  return true;
}

void UnloadFieldDimensionsClass(JNIEnv* env) {
  if (s_thudnerAutoFieldDimensionsClass) {
    env->DeleteGlobalRef(s_thudnerAutoFieldDimensionsClass);
    s_thudnerAutoFieldDimensionsClass = nullptr;
    s_thunderAutoFieldDimensionsConstructor = nullptr;
  }
}

jobject FieldDimensionsConstruct(JNIEnv* env, jdouble lengthMeters, jdouble widthMeters) {
  jobject dimensions = env->NewObject(s_thudnerAutoFieldDimensionsClass,
                                      s_thunderAutoFieldDimensionsConstructor, lengthMeters, widthMeters);
  return dimensions;
}

bool LoadThunderAutoSendableChooser_ChooserSelection_TypeClass(JNIEnv* env) {
  jclass thunderAutoSendableChooserSelectionTypeClass =
      env->FindClass(THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE);
  if (!thunderAutoSendableChooserSelectionTypeClass)
    return false;

  s_thunderAutoSendableChooserSelectionTypeClass =
      static_cast<jclass>(env->NewGlobalRef(thunderAutoSendableChooserSelectionTypeClass));

  env->DeleteLocalRef(thunderAutoSendableChooserSelectionTypeClass);

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
    thunder::driver::ThunderAutoSendableChooserSelectionType type) {
  switch (type) {
    using enum thunder::driver::ThunderAutoSendableChooserSelectionType;
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

  env->DeleteLocalRef(thunderAutoSendableChooserSelectionClass);

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

jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(
    JNIEnv* env,
    const thunder::driver::ThunderAutoSendableChooserSelection& selection) {
  jobject typeJObj = ThunderAutoSendableChooser_ChooserSelection_TypeGet(env, selection.type);
  jstring projectNameJStr = env->NewStringUTF(selection.projectName.c_str());
  jstring itemNameJStr = env->NewStringUTF(selection.itemName.c_str());

  jobject selectionJObj =
      ThunderAutoSendableChooser_ChooserSelectionConstruct(env, typeJObj, projectNameJStr, itemNameJStr);

  env->DeleteLocalRef(typeJObj);
  env->DeleteLocalRef(projectNameJStr);
  env->DeleteLocalRef(itemNameJStr);

  return selectionJObj;
}

bool LoadThunderAutoModeStep_TypeClass(JNIEnv* env) {
  jclass thunderAutoModeStepTypeClass = env->FindClass(THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE);
  if (!thunderAutoModeStepTypeClass)
    return false;

  s_thunderAutoModeStepTypeClass = static_cast<jclass>(env->NewGlobalRef(thunderAutoModeStepTypeClass));

  env->DeleteLocalRef(thunderAutoModeStepTypeClass);

  // UNKNOWN
  s_thunderAutoModeStepTypeUnknownField = env->GetStaticFieldID(
      s_thunderAutoModeStepTypeClass, "UNKNOWN", "L" THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE ";");
  if (!s_thunderAutoModeStepTypeUnknownField)
    return false;

  // ACTION
  s_thunderAutoModeStepTypeActionField = env->GetStaticFieldID(
      s_thunderAutoModeStepTypeClass, "ACTION", "L" THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE ";");
  if (!s_thunderAutoModeStepTypeActionField)
    return false;

  // TRAJECTORY
  s_thunderAutoModeStepTypeTrajectoryField = env->GetStaticFieldID(
      s_thunderAutoModeStepTypeClass, "TRAJECTORY", "L" THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE ";");
  if (!s_thunderAutoModeStepTypeTrajectoryField)
    return false;

  // BRANCH_BOOL
  s_thunderAutoModeStepTypeBranchBoolField = env->GetStaticFieldID(
      s_thunderAutoModeStepTypeClass, "BRANCH_BOOL", "L" THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE ";");
  if (!s_thunderAutoModeStepTypeBranchBoolField)
    return false;

  // BRANCH_SWITCH
  s_thunderAutoModeStepTypeBranchSwitchField = env->GetStaticFieldID(
      s_thunderAutoModeStepTypeClass, "BRANCH_SWITCH", "L" THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE ";");
  if (!s_thunderAutoModeStepTypeBranchSwitchField)
    return false;

  return true;
}

void UnloadThunderAutoModeStep_TypeClass(JNIEnv* env) {
  if (s_thunderAutoModeStepTypeClass) {
    env->DeleteGlobalRef(s_thunderAutoModeStepTypeClass);
    s_thunderAutoModeStepTypeUnknownField = nullptr;
    s_thunderAutoModeStepTypeActionField = nullptr;
    s_thunderAutoModeStepTypeTrajectoryField = nullptr;
    s_thunderAutoModeStepTypeBranchBoolField = nullptr;
    s_thunderAutoModeStepTypeBranchSwitchField = nullptr;
  }
}

jobject ThunderAutoModeStep_TypeGet(JNIEnv* env, thunder::driver::ThunderAutoModeStepType type) {
  switch (type) {
    using enum thunder::driver::ThunderAutoModeStepType;
    case ACTION:
      return env->GetStaticObjectField(s_thunderAutoModeStepTypeClass, s_thunderAutoModeStepTypeActionField);
    case TRAJECTORY:
      return env->GetStaticObjectField(s_thunderAutoModeStepTypeClass,
                                       s_thunderAutoModeStepTypeTrajectoryField);
    case BRANCH_BOOL:
      return env->GetStaticObjectField(s_thunderAutoModeStepTypeClass,
                                       s_thunderAutoModeStepTypeBranchBoolField);
    case BRANCH_SWITCH:
      return env->GetStaticObjectField(s_thunderAutoModeStepTypeClass,
                                       s_thunderAutoModeStepTypeBranchSwitchField);
    case UNKNOWN:
    default:
      return env->GetStaticObjectField(s_thunderAutoModeStepTypeClass, s_thunderAutoModeStepTypeUnknownField);
  }
}
