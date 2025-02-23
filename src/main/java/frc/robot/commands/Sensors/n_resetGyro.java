package frc.robot.commands.Sensors;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class n_resetGyro extends Command {

  private final DrivebaseSubsystem m_subsystem;

  private boolean db = false;

  public n_resetGyro(DrivebaseSubsystem subsystem) {
    m_subsystem = subsystem;

  }

  @Override
  public void initialize() {
    if (db == false) {
      db = true;
      m_subsystem.zeroGyro();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    db = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
