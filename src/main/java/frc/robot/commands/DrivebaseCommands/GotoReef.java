package frc.robot.commands.DrivebaseCommands;

import frc.robot.subsystems.FrontFacingCameraSubsystem;
import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class GotoReef extends Command {

  private final MecanumDrivebase _drivebase;
  private final FrontFacingCameraSubsystem _camera;
  private PIDController drivePID;
  private PIDController turnPID;

  private double kP = .01;
  private double kI = .01;
  private double kD = .1;

  private double kP2 = .01;
  private double kI2 = .01;
  private double kD2 = .1;

  private double heading;
  private double calculatedTurnSpeed;
  private double turnError;

  private String level;


  public GotoReef(MecanumDrivebase subsystem, FrontFacingCameraSubsystem camera, String level) {
    _drivebase = subsystem;
    _camera = camera;
    drivePID = new PIDController(kP, kI, kD);
    turnPID = new PIDController(kP2, kI2, kD2);
    this.level = level;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    drivePID.reset();
    turnPID.reset();

    drivePID.setIZone(.4);
    turnPID.setIZone(.4);
  }

  private void calculatePIDMovement(double heading) {
    turnError = heading - _drivebase.getGyroYaw();
    turnPID.setSetpoint(heading);
    calculatedTurnSpeed = turnPID.calculate(turnError);

    this._drivebase.goTo(0, 0, calculatedTurnSpeed);

    SmartDashboard.putNumber("Output", calculatedTurnSpeed);
  }

  @Override
  public void execute() {
    if (_camera.getTagID() != -1) {
      if (_camera.getTagID() == 18 || _camera.getTagID() == 7) {
        heading = 0;
        calculatePIDMovement(heading);
      } else if (_camera.getTagID() == 19 || _camera.getTagID() == 6) {
        heading = 45;
        calculatePIDMovement(heading);
      } else if (_camera.getTagID() == 20 || _camera.getTagID() == 11) {
        heading = 135;
        calculatePIDMovement(heading);
      } else if (_camera.getTagID() == 21 || _camera.getTagID() == 10) {
        heading = 180;
        calculatePIDMovement(heading);
      } else if (_camera.getTagID() == 22 || _camera.getTagID() == 9) {
        heading = -135;
        calculatePIDMovement(heading);
      } else if (_camera.getTagID() == 17|| _camera.getTagID() == 8) {
        heading = -45;
        calculatePIDMovement(heading);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
