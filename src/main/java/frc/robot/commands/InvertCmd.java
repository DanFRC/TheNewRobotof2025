package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class InvertCmd extends Command {

  private final Drivebase m_subsystem;

  public InvertCmd(Drivebase subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.invertDrive();
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
