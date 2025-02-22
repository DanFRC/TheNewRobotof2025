package frc.robot.commands;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TakeNoteCmd extends Command {

  private final ElevatorSubsystem _elevator;
  private final ArmPivotSubsystem _armPivot;
  private final PIDController elevatorPID;
  private final PIDController armPID;

  private boolean finished;
  private double heading;
  private double armHeading;

  private boolean runOnce;

  private double testVariable = 0;

  private Timer timer = new Timer();

  public TakeNoteCmd(ElevatorSubsystem subsystem, ArmPivotSubsystem _ArmPivotSubsystem) {
    _elevator = subsystem;
    _armPivot = _ArmPivotSubsystem;
    elevatorPID = new PIDController(ElevatorConstants.kP + 0.0002, .0015, ElevatorConstants.kD);
    armPID = new PIDController(ArmPivotConstants.kP, ArmPivotConstants.kI, ArmPivotConstants.kD);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("TestVar", testVariable);
    SmartDashboard.putBoolean("Finished Note?", finished);
    timer.reset();
    finished = false;
    runOnce = false;
    elevatorPID.reset();

    armHeading = ArmPivotConstants.kArmPivotDeadZoneMin;
    heading = 8400;
  }

  public void runOnceFunction() {

  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("TestVar", testVariable);
    SmartDashboard.putBoolean("Finished Note?", finished);
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putBoolean("RunOnce", runOnce);
    SmartDashboard.putNumber("Arm Error", armPID.getError());
    
    _armPivot.driveArm(armPID.calculate(_armPivot.getEncoder(), armHeading));

    if (Math.abs(armPID.getError()) < .2) {
      _elevator.driveElevator(elevatorPID.calculate(_elevator.getEncoder(), heading));
      testVariable = 1;
      timer.start();

        if (Math.abs(elevatorPID.getError()) < 2000) {
          if (timer.get() > 0.2 && timer.get() < 1) {
            testVariable = 2;
            heading = 5200;
          }
          if (timer.get() > 1 && timer.get() < 1.5) {
            testVariable = 3;
            finished = true;
            
          }
          if (timer.get() > 1.5 && timer.get() < 2) {
          }
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    finished = false;
    runOnce = false;
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
