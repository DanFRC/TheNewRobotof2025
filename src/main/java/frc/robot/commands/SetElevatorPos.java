package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPos extends Command {

  private final ElevatorSubsystem _elevator;
  private final PIDController elevatorPID;
  private final SlewRateLimiter elevatorSpeedLimiter;

  // VARS
  private double encoderPos;
  private String LEVEL;

  // Encoder Limits
  private double LOWERMAX = 0.05; // Please adjust these values!!!!! I HAVE NOT TUNED IT!!!
  private double UPPERMAX = 0.7; // Adjust this aswell!

  private double kP = 0.1;
  private double kI = 0.1;
  private double kD = 0.1;

  private double speedLimit = 1;

  // Completes command once the elevator's encoder reads this point or less!
  private double EndDeadZone = 0.4;

  private double error;

  private double output;

  // POSITIONS
  // THESE POSITIIONS ARE FOR THE REV THROUGH BORE ((ABSOLUTE) VERSION OF THE) ENCODER
  // GIVES VALUES IN USUALLY SMALLER NUMBERS
  private double LOWGOAL = 3 / 10;
  private double LOWREEF = 35 / 10;
  private double MIDREEF = 4 / 10;
  private double HIGHREEF = 5 / 10;
  private double NEUTRAL = 1 / 10;

  public SetElevatorPos(ElevatorSubsystem subsystem, String ReefLevel) {
    _elevator = subsystem;
    LEVEL = ReefLevel;

    this.elevatorPID = new PIDController(kP, kI, kD);
    this.elevatorSpeedLimiter = new SlewRateLimiter(speedLimit);

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {
    output = 0;
    elevatorPID.reset();
    elevatorPID.setIZone(7.5);
  }

  private void setElevator(String GOAL) {
    double goalPos;

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
    error = goalPos - encoderPos;
    elevatorPID.setSetpoint(goalPos);
    output = elevatorSpeedLimiter.calculate(elevatorPID.calculate(encoderPos * elevatorPID.calculate(encoderPos)));

    // This if statement wraps the encoder positions between the 2 values
    if (encoderPos > LOWERMAX && encoderPos < UPPERMAX) {
      _elevator.driveElevator(output);
    }
  }

  @Override
  public void execute() {
    // Public function inside the Elevator Subsystem
    encoderPos = _elevator.getEncoder();

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
    }

    // VISUALISATION
    SmartDashboard.putNumber("Elevator Error", error);
    SmartDashboard.putNumber("PID Calculated Output", output);
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (error <= EndDeadZone) {
      return true;
    } else {
      return false;
    }
  }
}
