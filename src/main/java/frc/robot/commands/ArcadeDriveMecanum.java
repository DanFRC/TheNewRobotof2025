package frc.robot.commands;

import frc.robot.subsystems.MecanumDrivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArcadeDriveMecanum extends Command {

  private final MecanumDrivebase _drivebase;

  // Define stuff
  private CommandXboxController thisController;
  private double driveSpeedx = 0;
  private double driveSpeedy = 0;
  private double turnSpeed = 0;

  public ArcadeDriveMecanum(MecanumDrivebase drivebase, CommandXboxController controller) {
    _drivebase = drivebase;
    thisController = controller;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    driveSpeedy = thisController.getLeftY();
    driveSpeedx = thisController.getLeftX();
    turnSpeed = thisController.getRightX();
    _drivebase.drive(driveSpeedx, driveSpeedy, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
