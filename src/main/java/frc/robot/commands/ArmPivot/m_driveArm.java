package frc.robot.commands.ArmPivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;

public class m_driveArm extends Command {

  private final ArmSubsystem _armSubsystem;

  private CommandXboxController LEVEL;

  public m_driveArm(ArmSubsystem subsystem, CommandXboxController controller) {
    _armSubsystem = subsystem;
    LEVEL = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //_armSubsystem.driveArm(LEVEL.getLeftY());
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

      return false;
  }
}
