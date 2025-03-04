package frc.robot.commands.ArmPivot;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class m_enableit extends Command {

  private final ArmSubsystem _arm;

  public m_enableit(ArmSubsystem subsystem) {
    _arm = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    _arm.enableIt();
    SmartDashboard.putNumber("runCommanded", 1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
