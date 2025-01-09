package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDrivebase extends SubsystemBase {

    // Initialize motors
    private VictorSPX _frontLeftMotor = new VictorSPX(Constants.DrivebaseContants.kLeftMotorPort1);
    private VictorSPX _rearLeftMotor = new VictorSPX(Constants.DrivebaseContants.kLeftMotorPort2);
    private VictorSPX _frontRightMotor = new VictorSPX(Constants.DrivebaseContants.kRightMotorPort1);
    private VictorSPX _rearRightMotor = new VictorSPX(Constants.DrivebaseContants.kRightMotorPort2);

    private double invert = 1;
    private double thisDriveSpeed = 0;
    private double thisDriveSpeedx = 0;
    private double thisDriveSpeedy = 0;
    private double thisTurnSpeed = 0;

    private final MecanumDrive _drivebase = new MecanumDrive(
    (double frontLeftSpeed) -> {
        _frontLeftMotor.set(ControlMode.PercentOutput, frontLeftSpeed);
    },
    (double rearLeftSpeed) -> {
        _rearLeftMotor.set(ControlMode.PercentOutput, rearLeftSpeed);
    },
    (double frontRightSpeed) -> {
        _frontRightMotor.set(ControlMode.PercentOutput, frontRightSpeed);
    },
    (double rearRightSpeed) -> {
        _rearRightMotor.set(ControlMode.PercentOutput, rearRightSpeed);
    });

    public MecanumDrivebase() {}
        
    public void drive(double xSpeed, double ySpeed, double turnSpeed) {

        // Inputs for Mecanum drive
        xSpeed *= invert;
        ySpeed *= invert;

        double frontLeftSpeed = ySpeed + xSpeed + turnSpeed;
        double rearLeftSpeed = ySpeed - xSpeed + turnSpeed;
        double frontRightSpeed = ySpeed - xSpeed - turnSpeed;
        double rearRightSpeed = ySpeed + xSpeed - turnSpeed;

        double maxSpeed = Math.max(
            Math.max(Math.abs(frontLeftSpeed), Math.abs(rearLeftSpeed)),
            Math.max(Math.abs(frontRightSpeed), Math.abs(rearRightSpeed))
        );

        // Scale all wheel speeds if the maximum exceeds 1
        if (maxSpeed > 1.0) {
            frontLeftSpeed /= maxSpeed;
            rearLeftSpeed /= maxSpeed;
            frontRightSpeed /= maxSpeed;
            rearRightSpeed /= maxSpeed;
        }

        // Set motor outputs
        _frontLeftMotor.set(ControlMode.PercentOutput, frontLeftSpeed);
        _rearLeftMotor.set(ControlMode.PercentOutput, rearLeftSpeed);
        _frontRightMotor.set(ControlMode.PercentOutput, frontRightSpeed);
        _rearRightMotor.set(ControlMode.PercentOutput, rearRightSpeed);

        // Help visualize data
        thisDriveSpeed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        thisDriveSpeedx = xSpeed;
        thisDriveSpeedy = ySpeed;
        thisTurnSpeed = turnSpeed;
    }

    public void invertDrive() {
        // Invert the drive direction
        invert *= -1;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Inverted", invert == -1);

        // Print stuff to the dashboard
        SmartDashboard.putNumber("Drive Speed", maxSpeed);
        SmartDashboard.putNumber("Speed X", thisDriveSpeedx);
        SmartDashboard.putNumber("Speed Y", thisDriveSpeedy);
        SmartDashboard.putNumber("Turn Speed", thisTurnSpeed);
    }
}
