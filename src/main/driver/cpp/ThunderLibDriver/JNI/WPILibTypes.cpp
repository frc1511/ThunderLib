#include "WPILibTypes.hpp"
#include "JavaTypes.hpp"

static jclass s_rotation2dClass = nullptr;
static jmethodID s_rotation2dConstructor = nullptr;

static jclass s_pose2dClass = nullptr;
static jmethodID s_pose2dConstructor = nullptr;

static jclass s_chassisSpeedsClass = nullptr;
static jmethodID s_chassisSpeedsConstructor = nullptr;

static jclass s_pairClass = nullptr;
static jmethodID s_pairConstructor = nullptr;

bool LoadRotation2dClass(JNIEnv* env) {
  jclass rotation2dClass = env->FindClass(WPIMATH_ROTATION2D_SIGNATURE);
  if (!rotation2dClass)
    return false;

  s_rotation2dClass = static_cast<jclass>(env->NewGlobalRef(rotation2dClass));

  // new Rotation2d(double valueRadians)
  s_rotation2dConstructor = env->GetMethodID(s_rotation2dClass, "<init>", "(D)V");
  if (!s_rotation2dConstructor)
    return false;

  return true;
}

void UnloadRotation2dClass(JNIEnv* env) {
  if (s_rotation2dClass) {
    env->DeleteGlobalRef(s_rotation2dClass);
    s_rotation2dClass = nullptr;
    s_rotation2dConstructor = nullptr;
  }
}

jobject Rotation2dConstruct(JNIEnv* env, jdouble valueRadians) {
  jobject rotation2d = env->NewObject(s_rotation2dClass, s_rotation2dConstructor, valueRadians);
  return rotation2d;
}

bool LoadPose2dClass(JNIEnv* env) {
  jclass pose2dClass = env->FindClass(WPIMATH_POSE2D_SIGNATURE);
  if (!pose2dClass)
    return false;

  s_pose2dClass = static_cast<jclass>(env->NewGlobalRef(pose2dClass));

  // new Pose2d(double xMeters, double yMeters, Rotation2d rotation)
  s_pose2dConstructor = env->GetMethodID(s_pose2dClass, "<init>", "(DDL" WPIMATH_ROTATION2D_SIGNATURE ";)V");
  if (!s_pose2dConstructor)
    return false;

  return true;
}

void UnloadPose2dClass(JNIEnv* env) {
  if (s_pose2dClass) {
    env->DeleteGlobalRef(s_pose2dClass);
    s_pose2dClass = nullptr;
    s_pose2dConstructor = nullptr;
  }
}

jobject Pose2dConstruct(JNIEnv* env, jdouble xMeters, jdouble yMeters, jobject rotation2d) {
  jobject pose2d = env->NewObject(s_pose2dClass, s_pose2dConstructor, xMeters, yMeters, rotation2d);
  return pose2d;
}

bool LoadChassisSpeedsClass(JNIEnv* env) {
  jclass chassisSpeedsClass = env->FindClass(WPIMATH_CHASSISSPEEDS_SIGNATURE);
  if (!chassisSpeedsClass)
    return false;

  s_chassisSpeedsClass = static_cast<jclass>(env->NewGlobalRef(chassisSpeedsClass));

  // new ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond)
  s_chassisSpeedsConstructor = env->GetMethodID(s_chassisSpeedsClass, "<init>", "(DDD)V");
  if (!s_chassisSpeedsConstructor)
    return false;

  return true;
}

void UnloadChassisSpeedsClass(JNIEnv* env) {
  if (s_chassisSpeedsClass) {
    env->DeleteGlobalRef(s_chassisSpeedsClass);
    s_chassisSpeedsClass = nullptr;
    s_chassisSpeedsConstructor = nullptr;
  }
}

jobject ChassisSpeedsConstruct(JNIEnv* env,
                               jdouble vxMetersPerSecond,
                               jdouble vyMetersPerSecond,
                               jdouble omegaRadiansPerSecond) {
  jobject chassisSpeeds = env->NewObject(s_chassisSpeedsClass, s_chassisSpeedsConstructor, vxMetersPerSecond,
                                         vyMetersPerSecond, omegaRadiansPerSecond);
  return chassisSpeeds;
}

bool LoadPairClass(JNIEnv* env) {
  jclass pairClass = env->FindClass(WPIMATH_PAIR_SIGNATURE);
  if (!pairClass)
    return false;

  s_pairClass = static_cast<jclass>(env->NewGlobalRef(pairClass));

  // new Pair(Object first, Object second)
  s_pairConstructor = env->GetMethodID(s_pairClass, "<init>",
                                       "(L" JAVA_LANG_OBJECT_SIGNATURE ";L" JAVA_LANG_OBJECT_SIGNATURE ";)V");
  if (!s_pairConstructor)
    return false;

  return true;
}

void UnloadPairClass(JNIEnv* env) {
  if (s_pairClass) {
    env->DeleteGlobalRef(s_pairClass);
    s_pairClass = nullptr;
    s_pairConstructor = nullptr;
  }
}

jobject PairConstruct(JNIEnv* env, jobject first, jobject second) {
  jobject pair = env->NewObject(s_pairClass, s_pairConstructor, first, second);
  return pair;
}
