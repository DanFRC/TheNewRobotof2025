package frc.robot.commands.Drivebase;

import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.FrontFacingCameraSubsystem;
import frc.robot.subsystems.Sensors.RearFacingCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class a_goToReef extends Command {

  private final DrivebaseSubsystem m_subsystem;
  private final FrontFacingCameraSubsystem _Camera;

  private double xDistance;
  private double yDistance;

  private double angle = 0;
  private double offset = 0.14;
  private int wantedTag;

  private double frontDistanceOutput = 0;
  private double LRDistanceOutput = 0;

  private PIDController drivePID = new PIDController(0.5, 0.3, 0.09);
  private PIDController drivePID2 = new PIDController(0.7, 0.4, 0.15);
  private PIDController turnPID = new PIDController(0.01, 0.008, 0.004);

  public a_goToReef(DrivebaseSubsystem subsystem, FrontFacingCameraSubsystem camera, int SpecificTag) {
    m_subsystem = subsystem;
    this._Camera = camera;
    wantedTag = SpecificTag;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    xDistance = _Camera.getDistanceToTag();
    angle = m_subsystem.getGyroYaw();
    drivePID.setIZone(0.2);
    drivePID2.setIZone(0.4);
    turnPID.setIZone(6);
    yDistance = _Camera.getYDistanceFromTag();
  }

  @Override
  public void execute() {
    xDistance = _Camera.getDistanceToTag();
    yDistance = _Camera.getYDistanceFromTag();

    if (wantedTag == -1) {
      if (_Camera.getTagID() == 10 || _Camera.getTagID() == 21) {
        angle = 180;
      } else if (_Camera.getTagID() == 20 || _Camera.getTagID() == 11) {
        angle = -120;
      } else if (_Camera.getTagID() == 19 || _Camera.getTagID() == 6) {
        angle = -60;
      } else if (_Camera.getTagID() == 18 || _Camera.getTagID() == 7) {
        angle = 0;
      } else if (_Camera.getTagID() == 17 || _Camera.getTagID() == 8) {
        angle = 60;
      } else if (_Camera.getTagID() == 22 || _Camera.getTagID() == 9) {
        angle = 120;
      }
    } else {
      if (wantedTag == 21 || wantedTag == 10) {
        angle = 180;
      } else if (wantedTag == 20 || wantedTag == 11) {
        angle = -120;
      } else if (wantedTag == 19 || wantedTag == 6) {
        angle = -60;
      } else if (wantedTag == 18 || wantedTag == 7) {
        angle = 0;
      } else if (wantedTag == 17 || wantedTag == 8) {
        angle = 60;
      } else if (wantedTag == 22 || wantedTag == 9) {
        angle = 120;
      }
    }


    if (m_subsystem.getReefSide() == "left") {
      offset = -0.14;
    } else if (m_subsystem.getReefSide() == "right") {
      offset = 0.14;
    }

    SmartDashboard.putNumber("Front Distance", xDistance);
    SmartDashboard.putNumber("LR Distance", yDistance);

    frontDistanceOutput = drivePID.calculate(xDistance, -0.19);
    LRDistanceOutput = drivePID2.calculate(yDistance, offset);

    SmartDashboard.putNumber("SpeedX", frontDistanceOutput);

    if (wantedTag != -1) {
      if (_Camera.getTagID() == wantedTag) {
        m_subsystem.drive(LRDistanceOutput, -frontDistanceOutput, -turnPID.calculate(m_subsystem.getGyroYaw(), angle), true, false);
      }
    } else {
        m_subsystem.drive(LRDistanceOutput, -frontDistanceOutput, -turnPID.calculate(m_subsystem.getGyroYaw(), angle), true, false);
    } 


  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
