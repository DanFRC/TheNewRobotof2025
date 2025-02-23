// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivebaseContants {
    // Drive Motor Ports for AM14U6? (Might be a different drive base??)
    public static final int kLeftMotorPort1 = 8;
    public static final int kLeftMotorPort2 = 7;
    public static final int kRightMotorPort1 = 6;
    public static final int kRightMotorPort2 = 5;
    public static final int kGyroPort = 7;

    public static double turnP = 0.025;
    public static double turnI = 0.005;
    public static double turnD = 0.005;

    public static double driveP = 0.025;
    public static double driveI = 0.005;
    public static double driveD = 0.005;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorPort = 4;
    public static final int kLimitSwitchPort = 0;
    public static final int kENC_A = 8;
    public static final int kENC_B = 9;

    public static double kP = 0.0005;
    public static double kI = 0.0005;
    public static double kD = 0.0001;

    public static final double kElevatorDeadZoneMax = 16000;
    public static final double kElevatorDeadZoneMin = 10;

    public static final double kElevatorMid = 8400;
    public static final double kElevatorLow = 4000;
    public static final double kElevatorDropper = 4000;

    public static final double kElevatorNeutral = 8400;

    public static final double kElevatorNoteUp = 7461;
    public static final double kElevatorNoteDown = 5800;
  }

  public static class ArmPivotConstants {
    public static final int kArmPivotMotorPort = 3;
    public static final int kEncoderPort = 1;

    public static final double kArmPivotDeadZoneMax = 0.166;
    public static final double kArmPivotDeadZoneMin = 0.666;

    public static final double kArmPivotMid = 0.24;
    public static final double kArmPivotLow = 0.24;
    public static final double kArmPivotDropper = 0.3;
    public static final double kArmPivotHigh = 0.21;

    public static double kP = 0.3;
    public static double kI = 0.2;
    public static double kD = 0.02;
  }

  public static class ButtonBoxConstants {
    public static final int kL4_L_Button = 8;
    public static final int kL4_R_Button = 7;

    public static final int kL3_L_Button = 10;
    public static final int kL3_R_Button = 11;

    public static final int kL2_L_Button = 5;
    public static final int kL2_R_Button = 6;

    public static final int kALG_34 = 12;
    public static final int kALG_23 = 9;
  }

  public static class AprilTagConstants {
    public static final double kCORAL_STATION_X = 1.5;
    public static final double kCORAL_STATION_Y = 0;
  }
}
