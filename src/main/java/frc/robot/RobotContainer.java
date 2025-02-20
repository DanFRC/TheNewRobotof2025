// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonBoxConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AUTO_DriveSeconds;
import frc.robot.commands.ArcadeDriveMecanum;
import frc.robot.commands.DriveArmPivot;
import frc.robot.commands.DriveElevator;
//import frc.robot.commands.Autos;
import frc.robot.commands.InvertCmd;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SetElevatorPos;
import frc.robot.commands.SetGoalSide;
import frc.robot.commands.SmartDriveCmd;
import frc.robot.commands.DrivebaseCommands.GotoReef;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FrontFacingCameraSubsystem;
import frc.robot.subsystems.MecanumDrivebase;
import frc.robot.subsystems.RearFacingCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger; // Useful later but not as of 29.01.25

public class RobotContainer {

  // Define Subsystems
  private final FrontFacingCameraSubsystem _cameraObject = new FrontFacingCameraSubsystem();
  private final RearFacingCamera _rearCameraObject = new RearFacingCamera();

  private final CommandJoystick _driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final ElevatorSubsystem _elevator = 
      new ElevatorSubsystem();

  private final ArmPivotSubsystem _ArmPivotSubsystem = 
      new ArmPivotSubsystem();

  private final CommandXboxController _manualSubsystemController = 
      new CommandXboxController(1);

  private final Joystick _driverButtonControls = 
      new Joystick(OperatorConstants.kDriverControllerPort);
  private final MecanumDrivebase _drivebase = new MecanumDrivebase();
  private final CommandJoystick _buttonBox = new CommandJoystick(2);

  public RobotContainer() {
    configureBindings();

    _drivebase.setDefaultCommand(new SmartDriveCmd(_drivebase, _driverController, _driverButtonControls, _rearCameraObject, _cameraObject));
    _elevator.setDefaultCommand(new DriveElevator(_elevator, _manualSubsystemController));
    _ArmPivotSubsystem.setDefaultCommand(new DriveArmPivot(_ArmPivotSubsystem, _manualSubsystemController));
  }

  private void configureBindings() {
    _driverController.button(3).onTrue(new ParallelCommandGroup(new ResetGyroCmd(_drivebase)));

    _driverController.button(1).whileTrue(new GotoReef(_drivebase, _cameraObject, "High Goal"));

      _buttonBox.button(ButtonBoxConstants.kL4_L_Button).onTrue(new ParallelCommandGroup( 
        new SetElevatorPos(_elevator, "High Reef", _driverController),
        new SetGoalSide(_drivebase, "left")
    ));

    _buttonBox.button(ButtonBoxConstants.kL3_L_Button).onTrue(new ParallelCommandGroup(
      new SetElevatorPos(_elevator, "Mid Reef", _driverController),
      new SetGoalSide(_drivebase, "left")
    ));

    _buttonBox.button(ButtonBoxConstants.kL2_L_Button).onTrue(new ParallelCommandGroup(
      new SetElevatorPos(_elevator, "Low Reef", _driverController),
      new SetGoalSide(_drivebase, "left")
    ));

    
    _buttonBox.button(ButtonBoxConstants.kL4_R_Button).onTrue(new ParallelCommandGroup( 
      new SetElevatorPos(_elevator, "High Reef", _driverController),
      new SetGoalSide(_drivebase, "right")
  ));

  _buttonBox.button(ButtonBoxConstants.kL3_R_Button).onTrue(new ParallelCommandGroup(
    new SetElevatorPos(_elevator, "Mid Reef", _driverController),
    new SetGoalSide(_drivebase, "right")
  ));

  _buttonBox.button(ButtonBoxConstants.kL2_R_Button).onTrue(new ParallelCommandGroup(
    new SetElevatorPos(_elevator, "Low Reef", _driverController),
    new SetGoalSide(_drivebase, "right")
  ));



    _buttonBox.button(ButtonBoxConstants.kALG_23).onTrue(new ParallelCommandGroup(new SetElevatorPos(_elevator, "Neutral", _driverController)));
    
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new AUTO_DriveSeconds(_drivebase, _cameraObject, _rearCameraObject, 1, 0.3, 0, 0, "DDRIVE")
    );
}
}
