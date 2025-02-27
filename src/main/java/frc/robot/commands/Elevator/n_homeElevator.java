package frc.robot.commands.Elevator;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class n_homeElevator extends Command {

  private final ElevatorSubsystem _elevator;
  private boolean finished = false;

  public n_homeElevator(ElevatorSubsystem ele) {
    _elevator = ele;
  }

  @Override
  public void initialize() {
    _elevator.resetHome();
    finished = true;
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    finished = false;
  }

  @Override
  public boolean isFinished() {
    if (finished == true) {
      return true;
    } else {
      return false;
    }
  }
}
