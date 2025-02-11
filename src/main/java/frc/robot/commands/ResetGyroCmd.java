package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetGyroCmd extends Command {

  private final MecanumDrivebase m_subsystem;

  private boolean db = false;

  public ResetGyroCmd(MecanumDrivebase subsystem) {
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
