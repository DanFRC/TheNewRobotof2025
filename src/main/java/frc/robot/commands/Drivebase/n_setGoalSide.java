package frc.robot.commands.Drivebase;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class n_setGoalSide extends Command {

  private final DrivebaseSubsystem m_subsystem;

  private String side;
  private boolean done = false;

  public n_setGoalSide(DrivebaseSubsystem subsystem, String side) {
    m_subsystem = subsystem;
    this.side = side;
  }

  @Override
  public void initialize() {
    m_subsystem.setReefSide(side);
    done = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    done = false;
  }

  @Override
  public boolean isFinished() {
    if (done == true) {
        return true;
    } else{
        return false;
    }
  }
}
