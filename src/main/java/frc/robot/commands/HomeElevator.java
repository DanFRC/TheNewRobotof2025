package frc.robot.commands;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeElevator extends Command {

  private final ElevatorSubsystem _elevator;
  private final ArmPivotSubsystem _armPivot;
  private final PIDController armPID;

  private double kP = ArmPivotConstants.kP;
  private double kI = ArmPivotConstants.kI;
  private double kD = ArmPivotConstants.kD;

  private boolean finished = false;

  public HomeElevator(ElevatorSubsystem subsystem, ArmPivotSubsystem arm) {
    _elevator = subsystem;
    _armPivot = arm;
    armPID = new PIDController(kP, kI, kD);

    addRequirements(subsystem, arm);
  }

  @Override
  public void initialize() {
    finished = false;
  }

  @Override
  public void execute() {

    _armPivot.driveArm(armPID.calculate(_armPivot.getEncoder(), ArmPivotConstants.kArmPivotDeadZoneMax));

    SmartDashboard.putNumber("Home Error", armPID.getError());

    if (_elevator.getLimitSwitch() == true && Math.abs(armPID.getError()) < 0.3) {
      _elevator.HomeElevator();
    } else if (_elevator.getLimitSwitch() == false) {
      finished = true;
    }
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
