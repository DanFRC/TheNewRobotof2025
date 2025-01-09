package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArcadeDrive extends Command {

  private final Drivebase _drivebase;

  // Define stuff
  private CommandXboxController thisController;
  private double driveSpeed = 0;
  private double turnSpeed = 0;

  public ArcadeDrive(Drivebase drivebase, CommandXboxController controller) {
    _drivebase = drivebase;
    thisController = controller;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    driveSpeed = thisController.getLeftY();
    turnSpeed = thisController.getLeftX();
    _drivebase.drive(driveSpeed, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
