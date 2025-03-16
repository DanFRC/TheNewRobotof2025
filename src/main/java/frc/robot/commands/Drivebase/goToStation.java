package frc.robot.commands.Drivebase;

import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.RearFacingCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class goToStation extends Command {

  private final DrivebaseSubsystem m_subsystem;
  private final RearFacingCamera _Camera;

  private double xDistance;
  private double yDistance;

  private double angle = 54;

  private double frontDistanceOutput = 0;
  private double LRDistanceOutput = 0;

  private PIDController drivePID = new PIDController(0.6, 0.3, 0.09);
  private PIDController drivePID2 = new PIDController(0.6, 0.55, 0.9);
  private PIDController turnPID = new PIDController(0.01, 0.008, 0.004);

  public goToStation(DrivebaseSubsystem subsystem, RearFacingCamera camera) {
    m_subsystem = subsystem;
    this._Camera = camera;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (_Camera.getTagID() == 2 || _Camera.getTagID() == 3) {
      xDistance = _Camera.getDistanceToTag();
      yDistance = _Camera.getYDistanceFromTag();
    }
    angle = m_subsystem.getGyroYaw();
    drivePID.setIZone(0.2);
    drivePID2.setIZone(0.4);
    turnPID.setIZone(6);
  }

  @Override
  public void execute() {
    if (_Camera.getTagID() == 2 || _Camera.getTagID() == 12) {
      xDistance = _Camera.getDistanceToTag();
      yDistance = _Camera.getDistanceToTag();
      yDistance = _Camera.getYDistanceFromTag();
      angle = 43;
    } else if (_Camera.getTagID() == 1 || _Camera.getTagID() == 13) {
      xDistance = _Camera.getDistanceToTag();
      yDistance = _Camera.getDistanceToTag();
      yDistance = _Camera.getYDistanceFromTag();
      angle = -43;
    }

    frontDistanceOutput = drivePID.calculate(xDistance, 0.456);
    LRDistanceOutput = drivePID2.calculate(yDistance, 0);

    SmartDashboard.putNumber("SpeedX", frontDistanceOutput);

    m_subsystem.drive(-LRDistanceOutput, frontDistanceOutput, -turnPID.calculate(m_subsystem.getGyroYaw(), angle), true, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
