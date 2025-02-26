package frc.robot.commands.Unused;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

public class Test extends Command {

  private final DrivebaseSubsystem m_subsystem;
  private boolean finished = false;

  public Test(DrivebaseSubsystem subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setTest();
    finished = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    finished = false;
  }

  @Override
  public boolean isFinished() {
    if (finished == true) {
      return true;
    } else{
      return false;
    }
  }
}
