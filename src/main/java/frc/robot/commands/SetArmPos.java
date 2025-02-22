package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetArmPos extends Command {

  private final ArmPivotSubsystem _arm;
  private final PIDController armPID;
  private final SlewRateLimiter armSpeedLimiter;
  private final CommandJoystick _joy;

  // VARS
  private String LEVEL;

  private double kP = ArmPivotConstants.kP;
  private double kI = ArmPivotConstants.kI;
  private double kD = ArmPivotConstants.kD;

  private double speedLimit = 1;

  // Completes command once the elevator's encoder reads this point or less!

  private double error;
  private double goalPos;
  private double output;

  // POSITIONS
  // THESE POSITIIONS ARE FOR THE REV THROUGH BORE RELATIVE VERSION OF THE ENCODER
  private double LOWGOAL = ArmPivotConstants.kArmPivotDropper;
  private double LOWREEF = ArmPivotConstants.kArmPivotLow;
  private double MIDREEF = ArmPivotConstants.kArmPivotMid;
  private double HIGHREEF = ArmPivotConstants.kArmPivotHigh;
  private double NEUTRAL = ArmPivotConstants.kArmPivotDeadZoneMax;

  public SetArmPos(ArmPivotSubsystem subsystem, String ReefLevel, CommandJoystick driverContrller) {
    _arm = subsystem;
    LEVEL = ReefLevel;
    _joy = driverContrller;

    this.armPID = new PIDController(kP, kI, kD);
    this.armSpeedLimiter = new SlewRateLimiter(speedLimit);

    addRequirements(subsystem);
  }



  @Override
  public void initialize() {
    output = 0;
    armPID.reset();

    goalPos = NEUTRAL;
  }

  private void setArm(String GOAL) {

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
    error = _arm.getEncoder() - goalPos;
    output = armPID.calculate(_arm.getEncoder(), goalPos);

    // This if statement wraps the encoder positions between the 2 values
    //if () {
      _arm.driveArm(output);
        
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
    } 
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
