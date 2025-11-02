#include "ThunderLibJNI.hpp"

#include <ThunderLibDriver/Auto/ThunderAutoTrajectory.hpp>

using namespace thunder::driver;

static jobject ConvertThunderAutoTrajectoryStateToJava(JNIEnv* env, const ThunderAutoTrajectoryState& state) {
  jdouble stateTimeSeconds = state.time.value();
  jobject statePoseRotation2d = Rotation2dConstruct(env, state.pose.Rotation().Radians().value());
  jobject statePose2d =
      Pose2dConstruct(env, state.pose.X().value(), state.pose.Y().value(), statePoseRotation2d);
  jobject stateChassisSpeeds = ChassisSpeedsConstruct(
      env, state.chassisSpeeds.vx.value(), state.chassisSpeeds.vy.value(), state.chassisSpeeds.omega.value());
  jdouble stateLinearVelocityMetersPerSecond = state.linearVelocity.value();
  jobject stateHeadingRotation2d = Rotation2dConstruct(env, state.heading.Radians().value());

  jobject trajectoryState =
      ThunderTrajectoryStateConstruct(env, stateTimeSeconds, statePose2d, stateChassisSpeeds,
                                      stateLinearVelocityMetersPerSecond, stateHeadingRotation2d);

  env->DeleteLocalRef(statePoseRotation2d);
  env->DeleteLocalRef(statePose2d);
  env->DeleteLocalRef(stateChassisSpeeds);
  env->DeleteLocalRef(stateHeadingRotation2d);

  return trajectoryState;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_delete
 * Signature: (J)V
 */
void Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1delete(JNIEnv* env,
                                                                          jclass,
                                                                          jlong trajectoryHandle) {
  if (!trajectoryHandle)
    return;

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  delete trajectory;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_sample
 * Signature: (JD)Lcom/thunder/lib/trajectory/ThunderTrajectoryState;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1sample(JNIEnv* env,
                                                                             jclass,
                                                                             jlong trajectoryHandle,
                                                                             jdouble timeSeconds) {
  if (!trajectoryHandle) {
    jobject defaultState = ThunderTrajectoryStateConstruct(env);
    return defaultState;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  ThunderAutoTrajectoryState state = trajectory->sample(units::second_t(timeSeconds));

  jobject trajectoryState = ConvertThunderAutoTrajectoryStateToJava(env, state);
  return trajectoryState;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getDurationSeconds
 * Signature: (J)D
 */
jdouble Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getDurationSeconds(
    JNIEnv* env,
    jclass,
    jlong trajectoryHandle) {
  if (!trajectoryHandle)
    return 0.0;

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  units::second_t duration = trajectory->getDuration();

  return duration.value();
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getInitialState
 * Signature: (J)Lcom/thunder/lib/trajectory/ThunderTrajectoryState;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getInitialState(
    JNIEnv* env,
    jclass,
    jlong trajectoryHandle) {
  if (!trajectoryHandle) {
    jobject defaultState = ThunderTrajectoryStateConstruct(env);
    return defaultState;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  ThunderAutoTrajectoryState state = trajectory->getInitialState();

  jobject trajectoryState = ConvertThunderAutoTrajectoryStateToJava(env, state);
  return trajectoryState;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getFinalState
 * Signature: (J)Lcom/thunder/lib/trajectory/ThunderTrajectoryState;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getFinalState(JNIEnv* env,
                                                                                    jclass,
                                                                                    jlong trajectoryHandle) {
  if (!trajectoryHandle) {
    jobject defaultState = ThunderTrajectoryStateConstruct(env);
    return defaultState;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  ThunderAutoTrajectoryState state = trajectory->getFinalState();

  jobject trajectoryState = ConvertThunderAutoTrajectoryStateToJava(env, state);
  return trajectoryState;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getStartActions
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getStartActions(
    JNIEnv* env,
    jclass,
    jlong trajectoryHandle) {
  jobject hashSet = HashSetConstruct(env);

  if (!trajectoryHandle) {
    return hashSet;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::unordered_set<std::string>& actions = trajectory->getStartActions();

  for (const std::string& action : actions) {
    jobject actionString = env->NewStringUTF(action.c_str());
    HashSetAdd(env, hashSet, actionString);
    env->DeleteLocalRef(actionString);
  }

  return hashSet;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getEndActions
 * Signature: (J)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getEndActions(JNIEnv* env,
                                                                                    jclass,
                                                                                    jlong trajectoryHandle) {
  jobject hashSet = HashSetConstruct(env);

  if (!trajectoryHandle) {
    return hashSet;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::unordered_set<std::string>& actions = trajectory->getEndActions();

  for (const std::string& action : actions) {
    jobject actionString = env->NewStringUTF(action.c_str());
    HashSetAdd(env, hashSet, actionString);
    env->DeleteLocalRef(actionString);
  }

  return hashSet;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getStopTimes
 * Signature: (J)Ljava/util/ArrayList;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getStopTimes(JNIEnv* env,
                                                                                   jclass,
                                                                                   jlong trajectoryHandle) {
  jobject arrayList = ArrayListConstruct(env);

  if (!trajectoryHandle) {
    return arrayList;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::map<units::second_t, std::unordered_set<std::string>> stopActions = trajectory->getStopActions();

  for (const auto& [stopTime, _] : stopActions) {
    jobject stopTimeDouble = DoubleConstruct(env, stopTime.value());
    ArrayListAdd(env, arrayList, stopTimeDouble);
    env->DeleteLocalRef(stopTimeDouble);
  }

  return arrayList;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getStopActions
 * Signature: (JD)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getStopActions(JNIEnv* env,
                                                                                     jclass,
                                                                                     jlong trajectoryHandle,
                                                                                     jdouble timeSeconds) {
  jobject hashSet = HashSetConstruct(env);

  if (!trajectoryHandle) {
    return hashSet;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::map<units::second_t, std::unordered_set<std::string>> stopActions = trajectory->getStopActions();

  auto it = stopActions.find(units::second_t(timeSeconds));
  if (it == stopActions.end()) {
    return hashSet;
  }

  const std::unordered_set<std::string>& actionsAtTime = it->second;
  for (const std::string& action : actionsAtTime) {
    jobject actionString = env->NewStringUTF(action.c_str());
    HashSetAdd(env, hashSet, actionString);
    env->DeleteLocalRef(actionString);
  }

  return hashSet;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getActionTimes
 * Signature: (J)Ljava/util/ArrayList;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getActionTimes(JNIEnv* env,
                                                                                     jclass,
                                                                                     jlong trajectoryHandle) {
  jobject arrayList = ArrayListConstruct(env);

  if (!trajectoryHandle) {
    return arrayList;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::multimap<units::second_t, std::string>& actions = trajectory->getActions();

  for (const auto& [actionTime, _] : actions) {
    jobject actionTimeDouble = DoubleConstruct(env, actionTime.value());
    ArrayListAdd(env, arrayList, actionTimeDouble);
    env->DeleteLocalRef(actionTimeDouble);
  }

  return arrayList;
}

/*
 * Class:     com_thunder_lib_jni_ThunderLibJNI
 * Method:    ThunderAutoTrajectory_getActionsAtTime
 * Signature: (JD)Ljava/util/HashSet;
 */
jobject Java_com_thunder_lib_jni_ThunderLibJNI_ThunderAutoTrajectory_1getActionsAtTime(JNIEnv* env,
                                                                                       jclass,
                                                                                       jlong trajectoryHandle,
                                                                                       jdouble timeSeconds) {
  jobject hashSet = HashSetConstruct(env);

  if (!trajectoryHandle) {
    return hashSet;
  }

  ThunderAutoTrajectory* trajectory = reinterpret_cast<ThunderAutoTrajectory*>(trajectoryHandle);
  const std::multimap<units::second_t, std::string>& actions = trajectory->getActions();

  auto range = actions.equal_range(units::second_t(timeSeconds));
  for (auto it = range.first; it != range.second; ++it) {
    const std::string& action = it->second;
    jobject actionString = env->NewStringUTF(action.c_str());
    HashSetAdd(env, hashSet, actionString);
    env->DeleteLocalRef(actionString);
  }

  return hashSet;
}
