package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class InvertCmd extends Command {

  private final MecanumDrivebase m_subsystem;

  public InvertCmd(MecanumDrivebase subsystem) {
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
