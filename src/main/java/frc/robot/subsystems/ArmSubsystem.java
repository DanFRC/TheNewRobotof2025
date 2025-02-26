package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {

    //Definitions
    private final VictorSPX _liftMotor = new VictorSPX(ArmPivotConstants.kArmPivotMotorPort);
    private PIDController normalPID = new PIDController(ArmPivotConstants.kP, ArmPivotConstants.kI, ArmPivotConstants.kD);
    private PIDController slowPID = new PIDController(1.6, 2.4, 0.5);
    private SlewRateLimiter smoother = new SlewRateLimiter(0.4);

    private double point;
    private boolean slowed;
    private boolean homed;
    // Elevator confirmed using a REV through bore encoder
    private final DutyCycleEncoder _encoder = new DutyCycleEncoder(ArmPivotConstants.kEncoderPort);


    public ArmSubsystem() {
        // do smth once
        homed = false;
        point = _encoder.get();
        slowed = false;
    }

    public void driveArm(double speed) {
        SmartDashboard.putNumber("liftSpeed", speed);
        if (speed < 0 && _encoder.get() >= ArmPivotConstants.kArmPivotDeadZoneMax) {
            _liftMotor.set(ControlMode.PercentOutput, speed);
            SmartDashboard.putBoolean("Up", true);
        } else if (speed > 0 && _encoder.get() <= ArmPivotConstants.kArmPivotDeadZoneMin) {
            _liftMotor.set(ControlMode.PercentOutput, speed);
        } else {
            _liftMotor.set(ControlMode.PercentOutput, 0);
        }

    }

    public void setArm(double setPoint, boolean slow) {
        if (slow == true) {
            slowPID.reset();
            slowPID.setIZone(0.2);
        } else {
            normalPID.reset();
            normalPID.setIZone(0.125);
        }
        point = setPoint;
        slowed = slow;
    }

    public double getArmError() {
        if (slowed == true) {
            return slowPID.getError();
        } else {
            return normalPID.getError();
        }
    }

    public double getEncoder() {
        return _encoder.get();
    }

    @Override
    public void periodic() {

            if (slowed == true) {
                _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(slowPID.calculate(_encoder.get(), point)));
            } else {
                _liftMotor.set(ControlMode.PercentOutput, smoother.calculate(normalPID.calculate(_encoder.get(), point)));
            }
        
        SmartDashboard.putNumber("Arm Encoder Output", getEncoder());
    }

    @Override
    public void simulationPeriodic() {
    }
}