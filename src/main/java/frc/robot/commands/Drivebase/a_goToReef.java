package frc.robot.commands.Drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.FrontFacingCameraSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class a_goToReef extends Command {

  private final DrivebaseSubsystem _drivebase;
  private final FrontFacingCameraSubsystem _camera;
  private PIDController drivePID;
  private PIDController turnPID;

  private CommandJoystick thisController;

  private double heading;
  private double calculatedTurnSpeed;
  private double turnError;
  private double output;

  double degree;
  double erir;  

  public a_goToReef(DrivebaseSubsystem subsystem, FrontFacingCameraSubsystem camera, CommandJoystick controller) {
    _drivebase = subsystem;
    _camera = camera;
    turnPID = new PIDController(Constants.DrivebaseContants.turnP, Constants.DrivebaseContants.turnI, Constants.DrivebaseContants.turnD);
    drivePID = new PIDController(Constants.DrivebaseContants.driveP, Constants.DrivebaseContants.driveI, Constants.DrivebaseContants.driveD);
    thisController = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    drivePID.reset();
    turnPID.reset();

    drivePID.setIZone(.4);
    turnPID.setIZone(.4);
  }

  private double findHeading(double TagID) {
    // Front
    if (TagID == 10 || TagID == 21) {
      return 0;
    // Close Right
    } else if (TagID == 11 || TagID == 20) {
      return 60;
    // Far Right
    } else if (TagID == 6 || TagID == 19) {
      return 120;
    // Back
    } else if (TagID == 7 || TagID == 18) {
      return 180;
    // Far Left
    } else if (TagID == 8 || TagID == 17) {
      return -120;
    // Close Left
    } else if (TagID == 9 || TagID == 22) {
      return -60;
    } else {
      return _drivebase.getGyroYaw();
    }
  }

  public static double wrapAngle(double angle) {
    angle = angle % 360;
    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }
    return angle;
}

  public double getOffset() {
    if (_drivebase.getReefSide() == "LEFT") {
      return -0.5;
    } else if (_drivebase.getReefSide() == "RIGHT") {
      return 0.5;
    } else {
      return 0;
    }
  }

  @Override
  public void execute() {

    if (_camera.getTagID() != -1) {
      if (_camera.getTagID() == 6 || _camera.getTagID() == 7 || _camera.getTagID() == 8 || _camera.getTagID() == 9 || _camera.getTagID() == 10 || _camera.getTagID() == 11 || _camera.getTagID() == 17 || _camera.getTagID() == 18 || _camera.getTagID() == 19 || _camera.getTagID() == 20 || _camera.getTagID() == 21 || _camera.getTagID() == 22) {

        if (_camera.getDistanceToTag() < 1.5) {
          degree = wrapAngle(findHeading(_camera.getTagID()));

          erir = wrapAngle(_drivebase.getGyroYaw() - degree);
          turnPID.setSetpoint(degree);
          output = -turnPID.calculate(erir);
          SmartDashboard.putNumber("erir", erir);
        } else {
          double turnError = _camera.getYDistanceFromTag();

          output = turnPID.calculate(turnError, 0);
        }

      } else {
      }
  

      // moves the robot toward the target
        double Xoutput;
        double Youtput;

        double XdistanceError = _camera.getDistanceToTag() - Constants.AprilTagConstants.kCORAL_STATION_X;
        double YdistanceError = _camera.getYDistanceFromTag() - Constants.AprilTagConstants.kCORAL_STATION_Y + getOffset();

        Xoutput = drivePID.calculate(XdistanceError, Constants.AprilTagConstants.kCORAL_STATION_X);
        Youtput = drivePID.calculate(YdistanceError, Constants.AprilTagConstants.kCORAL_STATION_Y);

        _drivebase.drive(-Xoutput, -Youtput, output, true, true);
    }
    

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
