package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class m_driveElevator extends Command {

  private final ElevatorSubsystem _elevator;

  private CommandXboxController LEVEL;

  public m_driveElevator(ElevatorSubsystem subsystem, CommandXboxController controller) {
    _elevator = subsystem;
    LEVEL = controller;

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    _elevator.driveElevator(-LEVEL.getRightY());
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

      return false;
  }
}
