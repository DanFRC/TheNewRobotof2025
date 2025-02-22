package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetElevatorEncoder extends Command {

  private final ElevatorSubsystem m_subsystem;

  private boolean db = false;

  public ResetElevatorEncoder(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;

  }

  @Override
  public void initialize() {
    if (db == false) {
      db = true;
      m_subsystem.resetEncoder();
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
