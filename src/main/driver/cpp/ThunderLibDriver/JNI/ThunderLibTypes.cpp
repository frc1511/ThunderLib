#include "ThunderLibTypes.hpp"
#include "WPILibTypes.hpp"

static jclass s_thunderTrajectoryStateClass = nullptr;
static jmethodID s_thunderTrajectoryStateDefaultConstructor = nullptr;
static jmethodID s_thunderTrajectoryStateConstructor = nullptr;

static jclass s_thunderAutoFieldSymmetryClass = nullptr;
static jfieldID s_thunderAutoFieldSymmetryNoneField = nullptr;
static jfieldID s_thunderAutoFieldSymmetryRotationalField = nullptr;
static jfieldID s_thunderAutoFieldSymmetryReflectionalField = nullptr;

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
