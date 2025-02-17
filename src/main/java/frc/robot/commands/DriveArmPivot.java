package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmPivotSubsystem;

public class DriveArmPivot extends Command {

  private final ArmPivotSubsystem _armSubsystem;

  private CommandXboxController LEVEL;

  public DriveArmPivot(ArmPivotSubsystem subsystem, CommandXboxController controller) {
    _armSubsystem = subsystem;
    LEVEL = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    _armSubsystem.driveArm(LEVEL.getLeftY());
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

      return false;
  }
}
