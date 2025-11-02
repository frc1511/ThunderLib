#pragma once

#include "jni.h"

// com.thunder.lib.trajectory.ThunderTrajectoryState

#define THUNDERLIB_TRAJECTORYSTATE_SIGNATURE "com/thunder/lib/trajectory/ThunderTrajectoryState"

bool LoadThunderTrajectoryStateClass(JNIEnv* env);
void UnloadThunderTrajectoryStateClass(JNIEnv* env);

jobject ThunderTrajectoryStateConstruct(JNIEnv* env);

jobject ThunderTrajectoryStateConstruct(JNIEnv* env,
                                        jdouble timeSeconds,
                                        jobject pose2d,
                                        jobject chassisSpeeds,
                                        jdouble linearVelocityMetersPerSecond,
                                        jobject headingRotation2d);
