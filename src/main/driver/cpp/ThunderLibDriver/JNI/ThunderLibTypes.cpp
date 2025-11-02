#include "ThunderLibTypes.hpp"
#include "WPILibTypes.hpp"

static jclass s_thunderTrajectoryStateClass = nullptr;
static jmethodID s_thunderTrajectoryStateDefaultConstructor = nullptr;
static jmethodID s_thunderTrajectoryStateConstructor = nullptr;

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
