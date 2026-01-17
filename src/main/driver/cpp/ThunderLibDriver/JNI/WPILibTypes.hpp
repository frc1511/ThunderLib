#pragma once

#include "jni.h"

// edu.wpi.first.math.geometry.Rotation2d

#define WPIMATH_ROTATION2D_SIGNATURE "edu/wpi/first/math/geometry/Rotation2d"

bool LoadRotation2dClass(JNIEnv* env);
void UnloadRotation2dClass(JNIEnv* env);

jobject Rotation2dConstruct(JNIEnv* env, jdouble valueRadians);

// edu.wpi.first.math.geometry.Pose2d

#define WPIMATH_POSE2D_SIGNATURE "edu/wpi/first/math/geometry/Pose2d"

bool LoadPose2dClass(JNIEnv* env);
void UnloadPose2dClass(JNIEnv* env);

jobject Pose2dConstruct(JNIEnv* env, jdouble xMeters, jdouble yMeters, jobject Rotation2d);

// edu.wpi.first.math.kinematics.ChassisSpeeds

#define WPIMATH_CHASSISSPEEDS_SIGNATURE "edu/wpi/first/math/kinematics/ChassisSpeeds"

bool LoadChassisSpeedsClass(JNIEnv* env);
void UnloadChassisSpeedsClass(JNIEnv* env);

jobject ChassisSpeedsConstruct(JNIEnv* env,
                               jdouble vxMetersPerSecond,
                               jdouble vyMetersPerSecond,
                               jdouble omegaRadiansPerSecond);

// edu.wpi.first.math.Pair

#define WPIMATH_PAIR_SIGNATURE "edu/wpi/first/math/Pair"

bool LoadPairClass(JNIEnv* env);
void UnloadPairClass(JNIEnv* env);

jobject PairConstruct(JNIEnv* env, jobject first, jobject second);
