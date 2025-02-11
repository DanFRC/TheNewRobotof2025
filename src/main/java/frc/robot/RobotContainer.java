// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonBoxConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDriveMecanum;
//import frc.robot.commands.Autos;
import frc.robot.commands.InvertCmd;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SmartDriveCmd;
import frc.robot.commands.DrivebaseCommands.GotoReef;
import frc.robot.subsystems.FrontFacingCameraSubsystem;
import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger; // Useful later but not as of 29.01.25

public class RobotContainer {

  // Define Subsystems
  private final FrontFacingCameraSubsystem _cameraObject = new FrontFacingCameraSubsystem();

  private final CommandJoystick _driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final Joystick _driverButtonControls = 
      new Joystick(OperatorConstants.kDriverControllerPort);
  private final MecanumDrivebase _drivebase = new MecanumDrivebase();
  private final CommandJoystick _buttonBox = new CommandJoystick(2);

  // Define Commands
  private final ArcadeDriveMecanum arcadeDriveCmd = new ArcadeDriveMecanum(_drivebase, _driverController);

  public RobotContainer() {
    configureBindings();

    _drivebase.setDefaultCommand(new SmartDriveCmd(_drivebase, _driverController, _driverButtonControls));
  }

  private void configureBindings() {

    _driverController.button(3).whileTrue(new ParallelCommandGroup(new InvertCmd(_drivebase)));
    _driverController.button(2).whileTrue(new ParallelCommandGroup(new ResetGyroCmd(_drivebase)));

    _buttonBox.button(ButtonBoxConstants.kL3_L_Button).whileTrue(new ParallelCommandGroup( 
      new GotoReef(_drivebase, _cameraObject, "L3_L")
    ));
  }

  public Command getAutonomousCommand() {
    return null;
    //return Autos.exampleAuto(m_exampleSubsystem);
  }
}
