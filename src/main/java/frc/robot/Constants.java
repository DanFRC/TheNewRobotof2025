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
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorPort = 4;
    public static final int kEncoderPort = 3;
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
}
