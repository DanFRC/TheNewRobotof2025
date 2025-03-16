package frc.robot.commands.ArmPivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
public class a_setArmPosition extends Command {

  private final ArmSubsystem _arm;
  private final ElevatorSubsystem _elevator;
  private final PIDController armPID;
  private final SlewRateLimiter armSpeedLimiter;
  private final CommandJoystick _joy;

  // VARS
  private String LEVEL;

  private double kP = ArmPivotConstants.kP;
  private double kI = ArmPivotConstants.kI;
  private double kD = ArmPivotConstants.kD;

  private double speedRate = 0.5;

  // Completes command once the elevator's encoder reads this point or less!

  private double error;
  private double goalPos;
  private double output;

  private double leanience = 0.066;

  private boolean finished = false;

  // POSITIONS
  // THESE POSITIIONS ARE FOR THE REV THROUGH BORE RELATIVE VERSION OF THE ENCODER
  private double LOWGOAL = ArmPivotConstants.kArmPivotDropper;

  private double LOWREEF = ArmPivotConstants.kArmPivotLow;

  private double MIDREEF = ArmPivotConstants.kArmPivotMid;

  private double HIGHREEF = ArmPivotConstants.kArmPivotHigh;
  private double HIGHSCORE = ArmPivotConstants.kArmPivotHighScore;

  private double NEUTRAL = ArmPivotConstants.kArmPivotNeutral;

  public a_setArmPosition(ArmSubsystem subsystem, ElevatorSubsystem esubsystem, String ReefLevel, CommandJoystick driverContrller) {
    _arm = subsystem;
    LEVEL = ReefLevel;
    _joy = driverContrller;
    _elevator = esubsystem;

    this.armPID = new PIDController(ArmPivotConstants.kP, ArmPivotConstants.kI, ArmPivotConstants.kD);
    this.armSpeedLimiter = new SlewRateLimiter(speedRate);

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {

    finished = false;
    output = 0;
    armPID.reset();

    armPID.setIZone(.05);

    goalPos = NEUTRAL;
  }

  private void setArm(String GOAL) {

    if (GOAL == "Low Goal") {
      goalPos = LOWGOAL;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Low Reef") {
      goalPos = LOWREEF;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Mid Reef") {
      goalPos = MIDREEF;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Algae") {
      goalPos = 0.5;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "High Reef") {
      goalPos = HIGHREEF;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Neutral") {
      goalPos = NEUTRAL;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "High Score") {
      if (_elevator.getLevel() == 4) {
        goalPos = .34;
      } else if (_elevator.getLevel() == 3) {
        goalPos = .34;
      } else if (_elevator.getLevel() == 2) {
        goalPos = .34;
      }
      _arm.setArm(goalPos, false, true);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else if (GOAL == "Intake") {
      goalPos = ArmPivotConstants.kArmPivotDeadZoneMin + leanience/1.5;
      _arm.setArm(goalPos, false, false);
      if (Math.abs(_arm.getArmError()) < leanience) {
        finished = true;
      }
    } else {
      goalPos = 0;
      return;
    }
    error = _arm.getEncoder() - goalPos;
    output = armPID.calculate(_arm.getEncoder(), goalPos);

    // This if statement wraps the encoder positions between the 2 values
    //if () {
        
      SmartDashboard.putNumber("PIDARM", output);
      SmartDashboard.putNumber("ARM Error", error);
      SmartDashboard.putNumber("ARMGOAL", goalPos);
    //}
  }

  @Override
  public void execute() {
    // Public function inside the Elevator Subsystem
    SmartDashboard.putString("ArmLevel", LEVEL);

    if (LEVEL == "Low Goal") {
      // GO TO LOWGOAL
      setArm(LEVEL);
    } else if (LEVEL == "Low Reef") {
      // GO TO LOWREEF
      setArm(LEVEL);
    } else if (LEVEL == "Mid Reef") {
      // GO TO MIDREEF
      setArm(LEVEL);
    } else if (LEVEL == "High Reef") {
      // GO TO HIGHREEF
      setArm(LEVEL);
    } else if (LEVEL == "Neutral") {
      // GO TO NEUTRAL
      setArm(LEVEL);
    } else if (LEVEL == "Intake") {
      // GO TO NEUTRAL
      setArm(LEVEL);
    } else if (LEVEL == "High Score") {
      setArm(LEVEL);
    } else if (LEVEL == "Algae") {
      setArm(LEVEL);
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
