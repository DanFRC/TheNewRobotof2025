package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.FrontFacingCameraSubsystem;
import frc.robot.subsystems.MecanumDrivebase;
import frc.robot.subsystems.RearFacingCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class SmartDriveCmd extends Command {

  private final MecanumDrivebase _drivebase;
  private final PIDController turnPID;
  private final PIDController drivePID;
  private final RearFacingCamera _camera;
  private final FrontFacingCameraSubsystem _wrong_camera;

  // Define stuff
  private CommandJoystick thisController;
  private Joystick thisbuttonControls;
  private double driveSpeedx = 0;
  private double driveSpeedy = 0;

  private double degree = 0;

  private double additiveTurn = 0;

  public SmartDriveCmd(MecanumDrivebase drivebase, CommandJoystick controller, Joystick buttonControls, RearFacingCamera camera, FrontFacingCameraSubsystem wrong_camera) {
    _drivebase = drivebase;
    thisController = controller;
    this.thisbuttonControls = buttonControls;
    this._camera = camera;
    this._wrong_camera = wrong_camera;

    turnPID = new PIDController(Constants.DrivebaseContants.turnP, Constants.DrivebaseContants.turnI, Constants.DrivebaseContants.turnD);
    drivePID = new PIDController(Constants.DrivebaseContants.driveP, Constants.DrivebaseContants.driveI, Constants.DrivebaseContants.driveD);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    turnPID.reset();
    turnPID.setIZone(.4);
    _drivebase.zeroGyro();
    additiveTurn = _drivebase.getGyroYaw();
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

  private double output = 0;
  private double eror = 0;

  private double onCrack = 0;

  @Override
  public void execute() {
    SmartDashboard.putNumber("GetThrottle", thisController.getThrottle());
    if (thisController.getThrottle() == -1) {
      driveSpeedy = thisController.getY();
      driveSpeedx = thisController.getX();

      SmartDashboard.putNumber("ControllerX", driveSpeedx);
      SmartDashboard.putNumber("ControllerY", driveSpeedy);
      SmartDashboard.putNumber("Gyro!!", _drivebase.getGyroYaw());
      SmartDashboard.putNumber("Needed Degree", degree);
      SmartDashboard.putNumber("Turn Output", output);
      SmartDashboard.putNumber("additiveTurn", additiveTurn);

      // thisController.button(3).whileTrue(Commands.runOnce(() -> {
      //   SmartDashboard.putNumber("Testie", 1);
      //   additiveTurn = 54;
      //   _drivebase.smartDrive(-driveSpeedx, -driveSpeedy, output, true);
      // }));
  
      if (thisbuttonControls.getRawButton(2) == true) {

        if (_camera.getTagID() == 1 || _camera.getTagID() == 2 || _camera.getTagID() == 12 || _camera.getTagID() == 13) {

          if (_camera.getDistanceToTag() < 1.5) {
            onCrack = 1;
            additiveTurn = 54;
          } else {
            onCrack = 0;
            double turnError = _camera.getYDistanceFromTag();

            additiveTurn = turnPID.calculate(turnError, 0);
          }

        } else {
          additiveTurn = 54;
        }

        // Turns the robot around until the rear camera sees the target
        if (_wrong_camera.getTagID() == 1 || _wrong_camera.getTagID() == 2 || _wrong_camera.getTagID() == 12 || _wrong_camera.getTagID() == 13) {
          while (!(_camera.getTagID() == 1 || _camera.getTagID() == 2 || _camera.getTagID() == 12 || _camera.getTagID() == 13)) {
            _drivebase.smartDrive(-driveSpeedx, -driveSpeedy, 0.5, true);
          }
    }
    

        // moves the robot toward the target
        if (_camera.getTagID() == 1 || _camera.getTagID() == 2 || _camera.getTagID() == 12 || _camera.getTagID() == 13) {
          double Xoutput;
          double Youtput;

          double XdistanceError = _camera.getDistanceToTag() - Constants.AprilTagConstants.kCORAL_STATION_X;
          double YdistanceError = _camera.getYDistanceFromTag() - Constants.AprilTagConstants.kCORAL_STATION_Y;

          Xoutput = drivePID.calculate(XdistanceError, Constants.AprilTagConstants.kCORAL_STATION_X);
          Youtput = drivePID.calculate(YdistanceError, Constants.AprilTagConstants.kCORAL_STATION_Y);

          if (onCrack == 1) {
            _drivebase.drive(-Xoutput, -Youtput, output);
          } else {
            _drivebase.drive(-Xoutput, -Youtput, additiveTurn);
          }
          
        } else {
          if (onCrack == 1) {
            _drivebase.smartDrive(-driveSpeedx, -driveSpeedy, output, true);
          } else {
            _drivebase.smartDrive(-driveSpeedx, -driveSpeedy, additiveTurn, true);
          }
        }
      } else {
        _drivebase.smartDrive(-driveSpeedx, -driveSpeedy, thisController.getTwist() , false);
      }
      
      degree = wrapAngle(additiveTurn);
  
      eror = wrapAngle(_drivebase.getGyroYaw() - degree);
      turnPID.setSetpoint(degree);
      output = -turnPID.calculate(eror);
      SmartDashboard.putNumber("Eror", eror);
      // Takes in four values, x speed, y speed, turns speed, and a Gyro Ange.
      
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
