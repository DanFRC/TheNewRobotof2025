package frc.robot.commands.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class a_setElevatorPosition extends Command {

  private final ElevatorSubsystem _elevator;
  private final Supplier<Double> _armEncoder_Supplier;
  private final PIDController elevatorPID;
  private final SlewRateLimiter elevatorSpeedLimiter;
  private final CommandJoystick _joy;

  // VARS
  private String LEVEL;
  private double armHeading;

  private double kP = ElevatorConstants.kP;
  private double kI = ElevatorConstants.kI;
  private double kD = ElevatorConstants.kD;

  private double speedRate = 0.5;

  // Completes command once the elevator's encoder reads this point or less!

  private double error;
  private double goalPos;
  private double output;

  private double leanience = 200;

  private boolean finished = false;

  // POSITIONS
  // THESE POSITIIONS ARE FOR THE REV THROUGH BORE RELATIVE VERSION OF THE ENCODER
  private double LOWGOAL = ElevatorConstants.kElevatorDropper;
  private double LOWREEF = ElevatorConstants.kElevatorLow;
  private double MIDREEF = ElevatorConstants.kElevatorMid;
  private double HIGHREEF = ElevatorConstants.kElevatorDeadZoneMax;
  private double NEUTRAL = ElevatorConstants.kElevatorNeutral;
  private double NOTEUP = ElevatorConstants.kElevatorNoteUp;
  private double NOTEDOWN = ElevatorConstants.kElevatorNoteDown;

  public a_setElevatorPosition(ElevatorSubsystem subsystem, ArmSubsystem armPivotSubsystem, String ReefLevel, CommandJoystick driverContrller) {
    _elevator = subsystem;
    LEVEL = ReefLevel;
    _joy = driverContrller;
    _armEncoder_Supplier = () -> armPivotSubsystem.getEncoder();

    this.elevatorPID = new PIDController(kP, kI, kD);
    this.elevatorSpeedLimiter = new SlewRateLimiter(speedRate);

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {
    finished = false;
    output = 0;
    elevatorPID.reset();
    elevatorPID.setIZone(1050);
  }

  private void setElevator(String GOAL) {

    if (GOAL == "Low Goal") {
      goalPos = LOWGOAL;
      _elevator.setElevator(goalPos);
      if (Math.abs(_elevator.getElevatorError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Low Reef") {
      goalPos = LOWREEF;
      _elevator.setElevator(goalPos);
      if (Math.abs(_elevator.getElevatorError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Mid Reef") {
      goalPos = MIDREEF;
      _elevator.setElevator(goalPos);
      if (Math.abs(_elevator.getElevatorError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "High Reef") {
      goalPos = HIGHREEF;
      _elevator.setElevator(goalPos);
      if (Math.abs(_elevator.getElevatorError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Neutral") {
      goalPos = NEUTRAL;
      _elevator.setElevator(goalPos);
      if (Math.abs(_elevator.getElevatorError()) < leanience) {
        finished = true;
      }
    }

  }

  @Override
  public void execute() {
    // Public function inside the Elevator Subsystem
    SmartDashboard.putString("ElevatorLevel", LEVEL);
    SmartDashboard.putNumber("ElevatorPID", output);
    SmartDashboard.putNumber("Elevator Error", error);
    SmartDashboard.putNumber("GOAL", goalPos);
    SmartDashboard.putNumber("Commanbd Elevator Reading", _elevator.getEncoder());
    error = _elevator.getEncoder() - goalPos;

    if (LEVEL == "Low Goal") {
      // GO TO LOWGOAL
      setElevator(LEVEL);
    } else if (LEVEL == "Low Reef") {
      // GO TO LOWREEF
      setElevator(LEVEL);
    } else if (LEVEL == "Mid Reef") {
      // GO TO MIDREEF
      setElevator(LEVEL);
    } else if (LEVEL == "High Reef") {
      // GO TO HIGHREEF
      setElevator(LEVEL);
    } else if (LEVEL == "Neutral") {
      // GO TO NEUTRAL
      setElevator(LEVEL);
    } else if (LEVEL == "Note Up") {
      // GO TO NEUTRAL
      setElevator(LEVEL);
    } else if (LEVEL == "Note Down") {
      // GO TO NEUTRAL
      setElevator(LEVEL);
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
