package frc.robot.commands.Drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.Sensors.FrontFacingCameraSubsystem;
import frc.robot.subsystems.Sensors.RearFacingCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class m_drive extends Command {
    private final DrivebaseSubsystem drivebase;
    private final PIDController turnController;
    private final PIDController driveController;
    private final RearFacingCamera rearCamera;
    private final FrontFacingCameraSubsystem frontCamera;

    private CommandJoystick controller;
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double targetAngle = 0;
    private double turnAdjustment = 0;
    
    private double turnOutput = 0;
    private double angleError = 0;

    public m_drive(DrivebaseSubsystem drivebase, CommandJoystick controller,
                         RearFacingCamera rearCamera, FrontFacingCameraSubsystem frontCamera) {
        this.drivebase = drivebase;
        this.controller = controller;
        this.rearCamera = rearCamera;
        this.frontCamera = frontCamera;
        turnController = new PIDController(Constants.DrivebaseContants.turnP, Constants.DrivebaseContants.turnI, Constants.DrivebaseContants.turnD);
        driveController = new PIDController(Constants.DrivebaseContants.driveP, Constants.DrivebaseContants.driveI, Constants.DrivebaseContants.driveD);
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setIZone(0.4);
        drivebase.zeroGyro();
        turnAdjustment = drivebase.getGyroYaw();
    }

    public static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("GetThrottle", controller.getThrottle());
        ySpeed = controller.getY();
        xSpeed = controller.getX();
        SmartDashboard.putNumber("ControllerX", xSpeed);
        SmartDashboard.putNumber("ControllerY", ySpeed);
        SmartDashboard.putNumber("GyroYaw", drivebase.getGyroYaw());
        SmartDashboard.putNumber("TargetAngle", targetAngle);
        SmartDashboard.putNumber("TurnOutput", turnOutput);
        SmartDashboard.putNumber("TurnAdjustment", turnAdjustment);
        
        drivebase.drive(-xSpeed, -ySpeed, controller.getTwist(), false, true);
        
        targetAngle = normalizeAngle(turnAdjustment);
        angleError = normalizeAngle(drivebase.getGyroYaw() - targetAngle);
        turnController.setSetpoint(targetAngle);
        turnOutput = -turnController.calculate(angleError);
        SmartDashboard.putNumber("AngleError", angleError);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
