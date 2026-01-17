#pragma once

#include "jni.h"
#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoSendableChooser.hpp>

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

// com.thunder.lib.trajectory.FieldDimensions

#define THUNDERLIB_FIELDDIMENSIONS_SIGNATURE "com/thunder/lib/trajectory/FieldDimensions"

bool LoadFieldDimensionsClass(JNIEnv* env);
void UnloadFieldDimensionsClass(JNIEnv* env);

jobject FieldDimensionsConstruct(JNIEnv* env, jdouble lengthMeters, jdouble widthMeters);

// com.thunder.lib.auto.ThunderAutoSendableChooser$ChooserSelection$Type

#define THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_TYPE_SIGNATURE \
  "com/thunder/lib/auto/ThunderAutoSendableChooser$ChooserSelection$Type"

bool LoadThunderAutoSendableChooser_ChooserSelection_TypeClass(JNIEnv* env);
void UnloadThunderAutoSendableChooser_ChooserSelection_TypeClass(JNIEnv* env);

jobject ThunderAutoSendableChooser_ChooserSelection_TypeGet(
    JNIEnv* env,
    thunder::driver::ThunderAutoSendableChooserSelectionType type);

// com.thunder.lib.auto.ThunderAutoSendableChooser$ChooserSelection

#define THUNDERLIB_THUNDERAUTOSENDABLECHOOSER_CHOOSERSELECTION_SIGNATURE \
  "com/thunder/lib/auto/ThunderAutoSendableChooser$ChooserSelection"

bool LoadThunderAutoSendableChooser_ChooserSelectionClass(JNIEnv* env);
void UnloadThunderAutoSendableChooser_ChooserSelectionClass(JNIEnv* env);

jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(JNIEnv* env);
jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(JNIEnv* env,
                                                             jobject type,
                                                             jstring projectName,
                                                             jstring itemName);

jobject ThunderAutoSendableChooser_ChooserSelectionConstruct(
    JNIEnv* env,
    const thunder::driver::ThunderAutoSendableChooserSelection& selection);

// com.thunder.lib.auto.ThunderAutoModeStep$Type

#define THUNDERLIB_THUNDERAUTOMODESTEP_TYPE_SIGNATURE "com/thunder/lib/auto/ThunderAutoModeStep$Type"

bool LoadThunderAutoModeStep_TypeClass(JNIEnv* env);
void UnloadThunderAutoModeStep_TypeClass(JNIEnv* env);

jobject ThunderAutoModeStep_TypeGet(JNIEnv* env, thunder::driver::ThunderAutoModeStepType type);
