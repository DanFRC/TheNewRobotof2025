// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.InvertCmd;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Define Subsystems
  private final CommandXboxController _driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Drivebase _drivebase = new Drivebase();

  // Define Commands
  private final ArcadeDrive arcadeDriveCmd = new ArcadeDrive(_drivebase, _driverController);

  public RobotContainer() {
    configureBindings();

    _drivebase.setDefaultCommand(arcadeDriveCmd);
  }

  private void configureBindings() {

    _driverController.x().whileTrue(new ParallelCommandGroup(
      new InvertCmd(_drivebase)
    ));
  }

  public Command getAutonomousCommand() {
    return null;
    //return Autos.exampleAuto(m_exampleSubsystem);
  }
}
