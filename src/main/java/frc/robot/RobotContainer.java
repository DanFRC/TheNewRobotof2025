// Rights Reserved: 2025, 10316 The Western Blue Tongues
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonBoxConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPivot.m_driveArm;
import frc.robot.commands.ArmPivot.a_setArmPosition;
import frc.robot.commands.Drivebase.a_driveSeconds;
import frc.robot.commands.Drivebase.n_setGoalSide;
import frc.robot.commands.Drivebase.m_drive;
import frc.robot.commands.Elevator.m_driveElevator;
import frc.robot.commands.Elevator.n_homeElevator;
import frc.robot.commands.Elevator.a_setElevatorPosition;
import frc.robot.commands.Elevator.a_takeNote;
import frc.robot.commands.Sensors.n_resetGyro;
import frc.robot.commands.Unused.ResetElevatorEncoder;
//import frc.robot.commands.DrivebaseCommands.GotoReef;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.FrontFacingCameraSubsystem;
import frc.robot.subsystems.Sensors.RearFacingCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {

  // Define Subsystems
  private final FrontFacingCameraSubsystem _cameraObject = new FrontFacingCameraSubsystem();
  private final RearFacingCamera _rearCameraObject = new RearFacingCamera();
  private final ElevatorSubsystem _elevator = new ElevatorSubsystem();
  private final ArmSubsystem _armPivot = new ArmSubsystem();
  private final DrivebaseSubsystem _drivebase = new DrivebaseSubsystem();

  // Define Controllers
  private final CommandJoystick _driver = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController _manualSubsystemController = new CommandXboxController(1);
  private final CommandJoystick _operator = new CommandJoystick(2);

  public RobotContainer() {
    configureBindings();

    //Drive the Robot
    _drivebase.setDefaultCommand(new m_drive(
      _drivebase,
      _driver,
      _rearCameraObject,
      _cameraObject
    ));

    // // Manually Drive The Elevator
    // _elevator.setDefaultCommand(new m_driveElevator(
    //   _elevator,
    //  _manualSubsystemController
    //  ));

    //  // Manually Drive The Arm
    // _armPivot.setDefaultCommand(new m_driveArm(
    //   _armPivot, 
    //   _manualSubsystemController
    //   ));
  }

  private void configureBindings() {
    _driver.button(11).onTrue(new ParallelCommandGroup(new n_resetGyro(
      _drivebase
      )));

  //_driver.button(1).whileTrue(new GotoReef(
    //_drivebase,
    //_cameraObject,
    //_driver
    //));

  
    _driver.button(8).onTrue(new n_homeElevator(
      _elevator
      ));

    // L-High
    _operator.button(ButtonBoxConstants.kL4_L_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
          "High Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
          "High Reef",
          _driver
          ),

        new n_setGoalSide(
          _drivebase,
          "left"
          )
    ));


    // L-Mid
    _operator.button(ButtonBoxConstants.kL3_L_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
          "Mid Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
          "Mid Reef",
          _driver
          ),
            
        new n_setGoalSide(
          _drivebase,
          "left"
          )
    ));


    // L-Low
    _operator.button(ButtonBoxConstants.kL2_L_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
           "Low Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
           "Low Reef",
          _driver
          ),

        new n_setGoalSide(
          _drivebase,
          "left"
          )
    ));

    // R-High
    _operator.button(ButtonBoxConstants.kL4_R_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
          "High Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
          "High Reef",
          _driver
          ),

        new n_setGoalSide(
          _drivebase,
          "right"
          )
    ));


    // R-Mid
    _operator.button(ButtonBoxConstants.kL3_R_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
          "Mid Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
          "Mid Reef",
          _driver
          ),

        new n_setGoalSide(
          _drivebase,
          "right"
          )
      ));


      // R-Low
    _operator.button(ButtonBoxConstants.kL2_R_Button).onTrue(
      new ParallelCommandGroup(
        new a_setElevatorPosition(
          _elevator,
          _armPivot,
          "Low Reef",
          _driver
          ),

        new a_setArmPosition(
          _armPivot,
           "Low Reef",
          _driver
          ),

        new n_setGoalSide(
          _drivebase, 
          "right"
          )
      ));


      //
      _operator.button(ButtonBoxConstants.kALG_23).onTrue(
        new ParallelCommandGroup(
          new ResetElevatorEncoder(
            _elevator
            )
        ));


    // Intake a coral piece
    _driver.button(13).onTrue(
      new SequentialCommandGroup(
        new a_setElevatorPosition(
          _elevator, 
          _armPivot,
          "Neutral", 
          _driver
          ),

          Commands.waitSeconds(1.5),
          
        new a_setArmPosition(
        _armPivot, 
        "Intake", 
        _driver
        ),

        Commands.waitSeconds(1.5),

        new a_setElevatorPosition(
          _elevator, 
          _armPivot, 
          "Intake", 
          _driver
          ),

          Commands.waitSeconds(1),

        new a_setElevatorPosition(
          _elevator, 
          _armPivot, 
          "Neutral", 
          _driver
          ),

          Commands.waitSeconds(0.8),

          new a_setArmPosition(
            _armPivot, 
            "Neutral", 
            _driver
            )
    ));


    // Interupt the Arm and Elevator
    _driver.button(5).onTrue(
      new ParallelCommandGroup(
        new m_driveElevator(
          _elevator, 
          _manualSubsystemController
          ),

        new m_driveArm(
          _armPivot, 
          _manualSubsystemController
          )
    ));

    // // Test works
    // _driver.button(12).onTrue(
    //   new SequentialCommandGroup(
    //     new Test(_drivebase),
    //     Commands.waitSeconds(1),
    //     (new Test(_drivebase))
    // ));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new a_driveSeconds(_drivebase, _cameraObject, _rearCameraObject, 1, 0.3, 0, 0, "DDRIVE")
    );
  }
}
