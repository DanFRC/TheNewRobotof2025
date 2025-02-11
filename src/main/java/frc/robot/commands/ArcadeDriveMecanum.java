package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class ArcadeDriveMecanum extends Command {

  private final MecanumDrivebase _drivebase;

  // Define stuff
  private CommandJoystick thisController;
  private double driveSpeedx = 0;
  private double driveSpeedy = 0;
  private double turnSpeed = 0;

  public ArcadeDriveMecanum(MecanumDrivebase drivebase, CommandJoystick controller) {
    _drivebase = drivebase;
    thisController = controller;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    driveSpeedy = thisController.getY();
    driveSpeedx = thisController.getX();
    turnSpeed = thisController.getTwist();
    // Takes in four values, x speed, y speed, turns speed, and a Gyro Ange.
    _drivebase.drive(driveSpeedx, driveSpeedy, turnSpeed);

    SmartDashboard.putNumber("ControllerX", driveSpeedx);
    SmartDashboard.putNumber("ControllerY", driveSpeedy);
    SmartDashboard.putNumber("Gyro!!", _drivebase.getGyroYaw());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
