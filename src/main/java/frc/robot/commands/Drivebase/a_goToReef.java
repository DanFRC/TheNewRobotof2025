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

  private double frontDistanceOutput = 0;
  private double LRDistanceOutput = 0;

  private PIDController drivePID = new PIDController(0.5, 0.3, 0.09);
  private PIDController drivePID2 = new PIDController(0.7, 0.4, 0.15);
  private PIDController turnPID = new PIDController(0.01, 0.008, 0.004);

  public a_goToReef(DrivebaseSubsystem subsystem, FrontFacingCameraSubsystem camera) {
    m_subsystem = subsystem;
    this._Camera = camera;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    xDistance = _Camera.getDistanceToTag();
    drivePID.setIZone(0.2);
    drivePID2.setIZone(0.4);
    turnPID.setIZone(6);
    yDistance = _Camera.getYDistanceFromTag();
  }

  @Override
  public void execute() {
    xDistance = _Camera.getDistanceToTag();
    yDistance = _Camera.getYDistanceFromTag();

    SmartDashboard.putNumber("Front Distance", xDistance);
    SmartDashboard.putNumber("LR Distance", yDistance);

    frontDistanceOutput = drivePID.calculate(xDistance, -0.26);
    LRDistanceOutput = drivePID2.calculate(yDistance, 0.14);

    SmartDashboard.putNumber("SpeedX", frontDistanceOutput);

    m_subsystem.drive(LRDistanceOutput, -frontDistanceOutput, -turnPID.calculate(m_subsystem.getGyroYaw(), 54), true, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
