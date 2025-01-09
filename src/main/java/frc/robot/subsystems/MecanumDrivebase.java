package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDrivebase extends SubsystemBase {

    // Initialise stuff
    private VictorSPX _leftMotor1 = new VictorSPX(Constants.DrivebaseContants.kLeftMotorPort1);
    private VictorSPX _leftMotor2 = new VictorSPX(Constants.DrivebaseContants.kLeftMotorPort2);
    private VictorSPX _rightMotor1 = new VictorSPX(Constants.DrivebaseContants.kRightMotorPort1);
    private VictorSPX _rightMotor2 = new VictorSPX(Constants.DrivebaseContants.kRightMotorPort2);

    private double invert = 1;
    private double thisDrivespeed = 0;
    private double thisTurnspeed = 0;

    // Define the drivebase
    private final DifferentialDrive _drivebase = new DifferentialDrive((double output) -> {
        _leftMotor1.set(ControlMode.PercentOutput,output);
        _leftMotor2.set(ControlMode.PercentOutput,output);
    },
    (double output2) -> {
        _rightMotor1.set(ControlMode.PercentOutput,-output2);
        _rightMotor2.set(ControlMode.PercentOutput,-output2);
    });

    public MecanumDrivebase() {
        // do smth once
    }

    public void drive(double driveSpeed, double turnSpeed) {
        // 2 given inputs drive the robot
        driveSpeed *= invert;
        _drivebase.arcadeDrive(driveSpeed, turnSpeed);

        // Help visualise data
        thisDrivespeed = driveSpeed;
        thisTurnspeed = turnSpeed;
    }
    
    public void invertDrive() {
        // Invert the drive
        invert *= -1;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (invert == 1) {
            SmartDashboard.putBoolean("Inverted", false);
        } else
        SmartDashboard.putBoolean("Inverted", true);

            // print stuff to the dashboard
        SmartDashboard.putNumber("Drive Speed", thisDrivespeed);
        SmartDashboard.putNumber("Turn Speed", thisTurnspeed);
    }
}
