// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDeadband = 0.05;
  }

  public static class DriveConstants {
    public static final double kMaxVelocity = 3.0; // meters per second
    public static final double kMaxAngularVelocity = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kModuleMaxTurningVelocity = Math.PI; // radians per second
    public static final double kModuleMaxTurningAcceleration = 2 * Math.PI; // radians per second squared

    public static final double kModuleSpacingX = 0.762; // meters
    public static final double kModuleSpacingY = 0.762; // meters

    public static final int kFLDriveMotorChannel = 1;
    public static final int kFLTurningMotorChannel = 2;
    public static final int kFRDriveMotorChannel = 3;
    public static final int kFRTurningMotorChannel = 4;
    public static final int kBLDriveMotorChannel = 5;
    public static final int kBLTurningMotorChannel = 6;
    public static final int kBRDriveMotorChannel = 7;
    public static final int kBRTurningMotorChannel = 8;

    public static final int kFLDriveEncoderChannelA = 1;
    public static final int kFLDriveEncoderChannelB = 2;
    public static final int kFLTurningEncoderChannelA = 3;
    public static final int kFLTurningEncoderChannelB = 4;

    public static final int kFRDriveEncoderChannelA = 5;
    public static final int kFRDriveEncoderChannelB = 6;
    public static final int kFRTurningEncoderChannelA = 7;
    public static final int kFRTurningEncoderChannelB = 8;

    public static final int kBLDriveEncoderChannelA = 9;
    public static final int kBLDriveEncoderChannelB = 10;
    public static final int kBLTurningEncoderChannelA = 11;
    public static final int kBLTurningEncoderChannelB = 12;

    public static final int kBRDriveEncoderChannelA = 13;
    public static final int kBRDriveEncoderChannelB = 14;
    public static final int kBRTurningEncoderChannelA = 15;
    public static final int kBRTurningEncoderChannelB = 16;

    public static final int kGyroChannel = 0;
  }

  public static final double kPeriodSeconds = 0.02;
}
