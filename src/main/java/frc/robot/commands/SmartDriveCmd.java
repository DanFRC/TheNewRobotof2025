package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class SmartDriveCmd extends Command {

  private final MecanumDrivebase _drivebase;
  private final PIDController turnPID;

  // Define stuff
  private CommandJoystick thisController;
  private Joystick thisbuttonControls;
  private double driveSpeedx = 0;
  private double driveSpeedy = 0;
  private double turnSpeed = 0;

  private double degree = 0;

  private double additiveTurn = 0;

  public SmartDriveCmd(MecanumDrivebase drivebase, CommandJoystick controller, Joystick buttonControls) {
    _drivebase = drivebase;
    thisController = controller;
    this.thisbuttonControls = buttonControls;

    turnPID = new PIDController(.01, 0.01, 0.1);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  public static double wrapAngle(double angle) {
    angle = angle % 360; // Keep within -360 to 360
    if (angle > 180) {
        angle -= 360; // Shift down to -180 to 180
    } else if (angle < -180) {
        angle += 360; // Shift up to -180 to 180
    }
    return angle;
}

  private double output = 0;
  private double eror = 0;

  @Override
  public void execute() {
    driveSpeedy = thisController.getY();
    driveSpeedx = thisController.getX();

    if (thisbuttonControls.getRawButton(1) == true) {
      additiveTurn = 45;
    } else {
      double twist = thisController.getTwist();
      additiveTurn += twist * (2.5 + Math.abs(twist));
    }
    
    degree = wrapAngle(additiveTurn);

    eror = _drivebase.getGyroYaw() - degree;
    turnPID.setSetpoint(degree);
    output = -turnPID.calculate(eror);
    SmartDashboard.putNumber("Eror", eror);
    // Takes in four values, x speed, y speed, turns speed, and a Gyro Ange.
    _drivebase.smartDrive(driveSpeedx, driveSpeedy, output);

    SmartDashboard.putNumber("ControllerX", driveSpeedx);
    SmartDashboard.putNumber("ControllerY", driveSpeedy);
    SmartDashboard.putNumber("Gyro!!", _drivebase.getGyroYaw());
    SmartDashboard.putNumber("Needed Degree", degree);
    SmartDashboard.putNumber("Turn Output", output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
