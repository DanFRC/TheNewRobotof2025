package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class SetGoalSide extends Command {

  private final MecanumDrivebase m_subsystem;

  private String side;
  private boolean done = false;

  public SetGoalSide(MecanumDrivebase subsystem, String side) {
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
