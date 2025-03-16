package frc.robot.commands.Drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.FrontFacingCameraSubsystem;
import frc.robot.subsystems.Sensors.RearFacingCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class a_driveSeconds extends Command {

  private final DrivebaseSubsystem _drivebase;

  // Define stuff
  private PIDController drivePID;
  private PIDController turnPID;

  private CommandJoystick thisController;
  private double driveSpeedx = 0;
  private double driveSpeedy = 0;
  private double heading = 0;

  private FrontFacingCameraSubsystem _FCam;
  private RearFacingCamera _RCam;

  private double DRIVING_TIME;

  private Timer timer = new Timer();

  private String DriveType;

  public a_driveSeconds(DrivebaseSubsystem drivebase, FrontFacingCameraSubsystem front_cam, RearFacingCamera rear_cam, double TIME, double xValue, double yValue, double turnHeading, String TYPE) {
    _drivebase = drivebase;
    this.driveSpeedx = xValue;
    this.driveSpeedy = yValue;
    this.heading = turnHeading;
    this.DRIVING_TIME = TIME;

    turnPID = new PIDController(0.0125, 0.0074, 0.006);
    drivePID = new PIDController(Constants.DrivebaseContants.driveP, Constants.DrivebaseContants.driveI, Constants.DrivebaseContants.driveD);

    this._FCam = front_cam;
    this._RCam = rear_cam;

    this.DriveType = TYPE;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  SlewRateLimiter limiter = new SlewRateLimiter(0.8);

  private double calculatePIDMovement(double thisheading, String TP) {
    if (TP == "KEEP-HEADING") {
      double turnError = _drivebase.getGyroYaw() - heading;
      double output = turnPID.calculate(_drivebase.getGyroYaw(), thisheading);

      return limiter.calculate(output);
    } else {
      return 0;
    }
  }

  @Override
  public void execute() {
    if (DriveType == "FO-HEADING") {
      _drivebase.drive(driveSpeedx, driveSpeedy, -calculatePIDMovement(heading, "KEEP-HEADING"), true, true);
    } else if (DriveType == "DDRIVE") {
      _drivebase.drive(driveSpeedx, driveSpeedy, heading, true, false);
    } else if (DriveType == "FO-DDRIVE") {
      _drivebase.drive(driveSpeedx, driveSpeedy, heading, true, true);
    }

    SmartDashboard.putNumber("Auto Drive Timer", timer.get());
    SmartDashboard.putString("Auto Drive Type", DriveType);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (timer.get() >= DRIVING_TIME) {
      return true;
    } else {
      return false;
    }
  }
}
