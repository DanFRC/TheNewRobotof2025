package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPos extends Command {

  private final ElevatorSubsystem _elevator;
  private final PIDController elevatorPID;
  private final SlewRateLimiter elevatorSpeedLimiter;
  private final CommandJoystick _joy;

  // VARS
  private String LEVEL;

  private double kP = 0.0001;
  private double kI = 0;
  private double kD = 0;

  private double speedLimit = 1;

  // Completes command once the elevator's encoder reads this point or less!

  private double error;
  private double goalPos;
  private double output;

  // POSITIONS
  // THESE POSITIIONS ARE FOR THE REV THROUGH BORE RELATIVE VERSION OF THE ENCODER
  private double LOWGOAL = ElevatorConstants.kElevatorDropper;
  private double LOWREEF = ElevatorConstants.kElevatorLow;
  private double MIDREEF = ElevatorConstants.kElevatorMid;
  private double HIGHREEF = ElevatorConstants.kElevatorDeadZoneMax;
  private double NEUTRAL = ElevatorConstants.kElevatorNeutral;

  public SetElevatorPos(ElevatorSubsystem subsystem, String ReefLevel, CommandJoystick driverContrller) {
    _elevator = subsystem;
    LEVEL = ReefLevel;
    _joy = driverContrller;

    this.elevatorPID = new PIDController(kP, kI, kD);
    this.elevatorSpeedLimiter = new SlewRateLimiter(speedLimit);

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {
    output = 0;
    elevatorPID.reset();

    goalPos = NEUTRAL;
  }

  private void setElevator(String GOAL) {

    if (GOAL == "Low Goal") {
      goalPos = LOWGOAL;
    } else if (GOAL == "Low Reef") {
      goalPos = LOWREEF;
    } else if (GOAL == "Mid Reef") {
      goalPos = MIDREEF;
    } else if (GOAL == "High Reef") {
      goalPos = HIGHREEF;
    } else if (GOAL == "Neutral") {
      goalPos = NEUTRAL;
    } else {
      goalPos = 0;
      return;
    }
    error = _elevator.getEncoder() - goalPos;
    elevatorPID.setSetpoint(goalPos);
    output = elevatorPID.calculate(error);

    // This if statement wraps the encoder positions between the 2 values
    //if () {
      _elevator.driveElevator(output);
      SmartDashboard.putNumber("ElevatorPID", output);
      SmartDashboard.putNumber("Elevator Error", error);
      SmartDashboard.putNumber("GOAL", goalPos);
    //}
  }

  @Override
  public void execute() {
    // Public function inside the Elevator Subsystem
    SmartDashboard.putString("ElevatorLevel", LEVEL);

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
    } 
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
