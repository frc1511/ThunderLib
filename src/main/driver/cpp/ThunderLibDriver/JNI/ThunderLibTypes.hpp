#pragma once

#include "jni.h"
#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>

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

// com.thunder.lib.trajectory.FieldSymmetry

#define THUNDERLIB_FIELDSYMMETRY_SIGNATURE "com/thunder/lib/trajectory/FieldSymmetry"

bool LoadFieldSymmetryClass(JNIEnv* env);
void UnloadFieldSymmetryClass(JNIEnv* env);

jobject FieldSymmetryGet(JNIEnv* env, thunder::driver::FieldSymmetry symmetry);
